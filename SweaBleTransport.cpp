#include "SweaBleTransport.h"

#ifdef SWEA_BLE

#include <NimBLEDevice.h>

// Development passkey. Production derives a per-device passkey from a factory secret (protocol doc, section 6).
#ifndef SWEA_BLE_PASSKEY
#define SWEA_BLE_PASSKEY 123456
#endif
#define SWEA_BLE_FW_STRING "swea-ble-0.2"
// RAM for the record ring (newest data is kept when it is full; the SD card keeps everything).
#ifndef SWEA_BLE_RING_BYTES
#define SWEA_BLE_RING_BYTES 24576
#endif

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
		OP_TIME_SYNC = 0x04,
		OP_SET_PROFILE = 0x07
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

	using swea::put16;
	using swea::put32;

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

	class DataCallbacks : public NimBLECharacteristicCallbacks
	{
		void onSubscribe(NimBLECharacteristic *c, ble_gap_conn_desc *desc, uint16_t subValue) override
		{
			if (g_owner) g_owner->onSubscribe(subValue & 1);	// bit 0 = notifications enabled
		}
	};

	ServerCallbacks g_serverCb;
	ControlCallbacks g_controlCb;
	DataCallbacks g_dataCb;
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

	drain();
	if (_connected && now - _lastStatusMs >= 1000)
	{
		_lastStatusMs = now;
		sendStatus(_encrypted);
	}
	return 0;
}

uint8_t SweaBleTransport::readControl(String &) { return 0; }	// control opcodes are handled in onControlWrite()

// Receives exactly the text packets that EmotiBit also writes to the SD card (one or more lines).
int8_t SweaBleTransport::sendData(const String &message)
{
	if (!_on || !_ringMem) return 0;
	const char *s = message.c_str();
	size_t len = message.length(), i = 0;
	while (i < len)
	{
		size_t j = i;
		while (j < len && s[j] != '\n') j++;
		size_t l = j - i;
		if (l > 0 && s[i + l - 1] == '\r') l--;
		if (l > 0) handleLine(s + i, l);
		i = j + 1;
	}
	return 0;
}

void SweaBleTransport::handleLine(const char *s, size_t len)
{
	if (!swea::parsePacketLine(s, len, _pp)) return;
	if (_pp.tag[0] == 'B' && _pp.tag[1] == '%')
	{
		float pct = _pp.v[_pp.n - 1];
		_batteryPct = pct < 0 ? 0 : (pct > 100 ? 100 : (uint8_t)(pct + 0.5f));
	}
	if (swea::profileAllows(_profile, _pp.tag)) _ring.push(_pp.tag, _pp.n, _pp.t_ms, _pp.v);
}

// Sends queued records as notifications while a bonded phone is subscribed. A few frames per call keep the main loop short.
void SweaBleTransport::drain()
{
	if (!_streaming || !_connected || !_encrypted || !_dataSubscribed) return;
	size_t cap = _mtu > 3 ? (size_t)_mtu - 3 : 20;
	if (cap > 244) cap = 244;
	uint8_t buf[244];
	for (int frame = 0; frame < 6; frame++)
	{
		swea::RecordRing::Cursor c = _ring.cursor();
		size_t n = swea::buildNotification(_ring, _seq, buf, cap);
		if (n == 0) break;
		if (!notifyData(buf, n))
		{
			_ring.restore(c);		// not sent: keep the records and try again later
			_notifyFailures++;
			break;
		}
		_seq++;
	}
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
	if (!_ringMem)
	{
		size_t bytes = SWEA_BLE_RING_BYTES;
		while (bytes >= 4096 && !(_ringMem = (uint8_t *)malloc(bytes))) bytes /= 2;	// shrink if the heap is tight
		if (_ringMem) _ring.init(_ringMem, bytes);
		else Serial.println("WARNING: no memory for the BLE record ring; data is not forwarded");
	}
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
	g_data->setCallbacks(&g_dataCb);
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
	if (_ringMem) { free(_ringMem); _ringMem = nullptr; }
	g_server = nullptr; g_data = g_control = g_status = nullptr;
}

bool SweaBleTransport::notifyData(const uint8_t *buf, size_t len)
{
	if (!g_data || !_connected || !_encrypted) return false;
	if (len > (size_t)(_mtu - 3)) return false;
	// NimBLE-Arduino 1.4.3 notify() returns nothing, so call the host API directly: it reports
	// back-pressure (out of mbufs) as an error and the caller keeps the records for the next try.
	os_mbuf *om = ble_hs_mbuf_from_flat(buf, len);
	if (!om) return false;
	return ble_gattc_notify_custom(_connHandle, g_data->getHandle(), om) == 0;	// takes ownership of om
}

void SweaBleTransport::sendStatus(bool notify)
{
	if (!g_status) return;
	uint8_t s[20] = {0};
	s[0] = PROTOCOL_VERSION;
	s[1] = !_on ? 0 : (_streaming ? 3 : (_connected ? 2 : 1));
	s[2] = (_batteryPct != 0xFF && _batteryPct < 20) ? 0x08 : 0;	// bit3 low battery; recording/SD/gap/pairing flags come in Phase 4
	s[3] = _batteryPct;						// 0xFF = unknown
	s[4] = _profile;
	s[5] = g_bootCount;
	put16(&s[6], _seq);
	put16(&s[8], 0);						// oldest_seq: no ring buffer yet
	put16(&s[10], (uint16_t)_ring.dropped());
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
void SweaBleTransport::onConnected(uint16_t connHandle) { _connHandle = connHandle; _connected = true; _encrypted = false; _dataSubscribed = false; _mtu = 23; }
void SweaBleTransport::onDisconnected() { _connected = false; _encrypted = false; _streaming = false; _dataSubscribed = false; _connHandle = 0xFFFF; }
void SweaBleTransport::onEncryptionChange(bool encrypted) { _encrypted = encrypted; }
void SweaBleTransport::onMtu(uint16_t mtu) { _mtu = mtu; }
void SweaBleTransport::onSubscribe(bool subscribed) { _dataSubscribed = subscribed; }

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
		if (len != 2 || data[1] > 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		_profile = data[1];
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
	case OP_SET_PROFILE:
		if (len != 2 || data[1] > 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		_profile = data[1];
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	default:
		sendControlReply(op, ST_UNSUPPORTED, nullptr, 0);
	}
}

#endif // SWEA_BLE
