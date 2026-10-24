#include "SweaBleTransport.h"

#ifdef SWEA_BLE

#include <NimBLEDevice.h>

// Development passkey. Production derives a per-device passkey from a factory secret (protocol doc, section 6).
#ifndef SWEA_BLE_PASSKEY
#define SWEA_BLE_PASSKEY 123456
#endif
#define SWEA_BLE_FW_STRING "swea-ble-0.1"

namespace
{
	const char *SERVICE_UUID = "3f5737c4-b391-44c7-bd74-f2ee561c9775";
	const char *DATA_UUID = "f14e01ab-48d0-461a-b54e-49fd7be9901a";
	const char *CONTROL_UUID = "1118282f-3dfe-44a7-a12f-6d06987f1092";
	const char *STATUS_UUID = "8c2502e8-eacd-45d1-bd20-361eb5a762a2";

	enum Opcode : uint8_t
	{
		OP_GET_INFO = 0x01,
		OP_START_STREAM = 0x02,
		OP_STOP_STREAM = 0x03,
		OP_TIME_SYNC = 0x04
	};
	enum ReplyStatus : uint8_t
	{
		ST_OK = 0,
		ST_BAD_ARGS = 1,
		ST_NOT_ALLOWED = 2,
		ST_BUSY = 3,
		ST_UNSUPPORTED = 4
	};

	NimBLEServer *g_server = nullptr;
	NimBLECharacteristic *g_data = nullptr;
	NimBLECharacteristic *g_control = nullptr;
	NimBLECharacteristic *g_status = nullptr;
	SweaBleTransport *g_owner = nullptr;
	uint8_t g_bootCount = 0;

	void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
	void put32(uint8_t *p, uint32_t v) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; p[2] = (v >> 16) & 0xFF; p[3] = v >> 24; }

	class ServerCallbacks : public NimBLEServerCallbacks
	{
		void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override
		{
			if (g_owner) g_owner->onConnected(desc->conn_handle);
			// Ask for encryption right away; data is only sent on an encrypted link.
			NimBLEDevice::startSecurity(desc->conn_handle);
		}
		void onDisconnect(NimBLEServer *server) override
		{
			if (g_owner) g_owner->onDisconnected();
			NimBLEDevice::startAdvertising();
		}
		void onMTUChange(uint16_t mtu, ble_gap_conn_desc *desc) override
		{
			if (g_owner) g_owner->onMtu(mtu);
		}
		uint32_t onPassKeyRequest() override { return SWEA_BLE_PASSKEY; }
		void onAuthenticationComplete(ble_gap_conn_desc *desc) override
		{
			if (g_owner) g_owner->onEncryptionChange(desc->sec_state.encrypted);
			if (!desc->sec_state.encrypted)
			{
				NimBLEDevice::getServer()->disconnect(desc->conn_handle);	// failed pairing: drop the link
			}
		}
	};

	class ControlCallbacks : public NimBLECharacteristicCallbacks
	{
		void onWrite(NimBLECharacteristic *c) override
		{
			std::string v = c->getValue();
			if (g_owner) g_owner->onControlWrite((const uint8_t *)v.data(), v.size());
		}
	};

	ServerCallbacks g_serverCb;
	ControlCallbacks g_controlCb;
}

// ---------------------------------------------------------------------------------------
uint8_t SweaBleTransport::begin(int32_t, uint8_t, uint16_t)
{
	if (!_on) startBle();
	return 0;	// SUCCESS
}

void SweaBleTransport::end()
{
	if (_on) stopBle();
}

int8_t SweaBleTransport::disconnect()
{
	if (g_server && _connected) g_server->disconnect(_connHandle);
	return 0;
}

int8_t SweaBleTransport::update(String &, uint16_t &)
{
	if (!_on) return 0;
	uint32_t now = millis();

	// Phase 2 test frame: one debug record per second while a bonded phone has started the stream.
	if (_streaming && _connected && _encrypted && now - _lastTestFrameMs >= 1000)
	{
		_lastTestFrameMs = now;
		uint8_t buf[3 + 2 + 1 + 4 + 4];
		buf[0] = PROTOCOL_VERSION;
		put16(&buf[1], _seq++);
		buf[3] = 'D'; buf[4] = 'B';
		buf[5] = 1;
		put32(&buf[6], now);
		float v = (float)_seq;
		memcpy(&buf[10], &v, 4);
		notifyData(buf, sizeof(buf));
	}
	if (_connected && now - _lastStatusMs >= 1000)
	{
		_lastStatusMs = now;
		sendStatus(_encrypted);
	}
	return 0;
}

uint8_t SweaBleTransport::readControl(String &) { return 0; }	// control opcodes are handled in onControlWrite()

int8_t SweaBleTransport::sendData(const String &)
{
	// Phase 3: convert the text packets into binary records and queue them here.
	return 0;
}

int8_t SweaBleTransport::addCredential(const String &, const String &, const String &, const String &) { return 0; }
void SweaBleTransport::setDeviceId(const String emotibitDeviceId) { _deviceId = emotibitDeviceId; }
bool SweaBleTransport::isConnected() { return _connected && _encrypted; }
bool SweaBleTransport::isOff() { return !_on; }
void SweaBleTransport::updateStatus() {}
uint8_t SweaBleTransport::status(bool) { return _on ? WL_CONNECTED : WL_DISCONNECTED; }
uint8_t SweaBleTransport::getNumCredentials() { return 0; }
bool SweaBleTransport::isEnterpriseNetworkListed() { return false; }
uint8_t SweaBleTransport::getMaxNumCredentialAllowed() { return MAX_CREDENTIALS; }

// ---------------------------------------------------------------------------------------
void SweaBleTransport::startBle()
{
	g_owner = this;
	g_bootCount++;
	String suffix = _deviceId.length() >= 4 ? _deviceId.substring(_deviceId.length() - 4) : String("0000");
	String name = "EmotiBit-" + suffix;

	NimBLEDevice::init(name.c_str());
	NimBLEDevice::setMTU(247);
	NimBLEDevice::setSecurityAuth(true, true, true);			// bonding, MITM protection, LE Secure Connections
	NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);	// passkey is shown on the label / QR code
	NimBLEDevice::setSecurityPasskey(SWEA_BLE_PASSKEY);

	g_server = NimBLEDevice::createServer();
	g_server->setCallbacks(&g_serverCb);
	NimBLEService *svc = g_server->createService(SERVICE_UUID);

	g_data = svc->createCharacteristic(DATA_UUID, NIMBLE_PROPERTY::NOTIFY);
	g_control = svc->createCharacteristic(CONTROL_UUID,
		NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN | NIMBLE_PROPERTY::NOTIFY);
	g_control->setCallbacks(&g_controlCb);
	g_status = svc->createCharacteristic(STATUS_UUID,
		NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN | NIMBLE_PROPERTY::NOTIFY);
	svc->start();

	NimBLEAdvertising *adv = NimBLEDevice::getAdvertising();
	adv->addServiceUUID(SERVICE_UUID);	// advertising carries only the service UUID and the short name
	adv->setScanResponse(true);
	adv->start();

	_on = true;
	_connected = _encrypted = _streaming = false;
	sendStatus(false);
	Serial.print("SWEA BLE advertising as "); Serial.println(name);
#if SWEA_BLE_PASSKEY == 123456
	Serial.println("WARNING: development passkey in use");
#endif
}

void SweaBleTransport::stopBle()
{
	_on = false;
	_connected = _encrypted = _streaming = false;
	NimBLEDevice::deinit(true);
	g_server = nullptr; g_data = g_control = g_status = nullptr;
}

void SweaBleTransport::notifyData(const uint8_t *buf, size_t len)
{
	if (!g_data || !_connected || !_encrypted) return;
	if (len > (size_t)(_mtu - 3)) { _dropped++; return; }
	g_data->setValue(buf, len);
	g_data->notify();
}

void SweaBleTransport::sendStatus(bool notify)
{
	if (!g_status) return;
	uint8_t s[20] = {0};
	s[0] = PROTOCOL_VERSION;
	s[1] = !_on ? 0 : (_streaming ? 3 : (_connected ? 2 : 1));
	s[2] = 0;								// flags: recording/SD/gap/low battery/pairing are filled in Phase 3/4
	s[3] = 0xFF;							// battery percent: unknown in Phase 2
	s[4] = 0;								// profile
	s[5] = g_bootCount;
	put16(&s[6], _seq);
	put16(&s[8], 0);						// oldest_seq: no ring buffer yet
	put16(&s[10], (uint16_t)_dropped);
	put32(&s[12], millis());
	put16(&s[16], (uint16_t)(ESP.getFreeHeap() / 1024));
	g_status->setValue(s, sizeof(s));
	if (notify && _connected) g_status->notify();
}

void SweaBleTransport::sendControlReply(uint8_t opcode, uint8_t status, const uint8_t *args, size_t n)
{
	if (!g_control || !_connected) return;
	uint8_t buf[40];
	if (n > sizeof(buf) - 2) n = sizeof(buf) - 2;
	buf[0] = 0x80 | opcode;
	buf[1] = status;
	if (n) memcpy(&buf[2], args, n);
	g_control->setValue(buf, n + 2);
	g_control->notify();
}

// ---- BLE callbacks -------------------------------------------------------------------
void SweaBleTransport::onConnected(uint16_t connHandle) { _connHandle = connHandle; _connected = true; _encrypted = false; _mtu = 23; }
void SweaBleTransport::onDisconnected() { _connected = false; _encrypted = false; _streaming = false; _connHandle = 0xFFFF; }
void SweaBleTransport::onEncryptionChange(bool encrypted) { _encrypted = encrypted; }
void SweaBleTransport::onMtu(uint16_t mtu) { _mtu = mtu; }

void SweaBleTransport::onControlWrite(const uint8_t *data, size_t len)
{
	if (len < 1) return;
	uint8_t op = data[0];
	if (!_encrypted)
	{
		sendControlReply(op, ST_NOT_ALLOWED, nullptr, 0);
		return;
	}
	switch (op)
	{
	case OP_GET_INFO:
	{
		uint8_t out[40];
		size_t n = 0;
		out[n++] = PROTOCOL_VERSION;
		const char *fw = SWEA_BLE_FW_STRING;
		for (size_t i = 0; fw[i] && n < sizeof(out) - 2; i++) out[n++] = fw[i];
		out[n++] = 0;
		sendControlReply(op, ST_OK, out, n);
		break;
	}
	case OP_START_STREAM:
		if (len != 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		_streaming = true;
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	case OP_STOP_STREAM:
		_streaming = false;
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	case OP_TIME_SYNC:
	{
		if (len != 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		uint8_t out[5];
		out[0] = data[1];
		put32(&out[1], millis());
		sendControlReply(op, ST_OK, out, sizeof(out));
		break;
	}
	default:
		sendControlReply(op, ST_UNSUPPORTED, nullptr, 0);
	}
}

#endif // SWEA_BLE
