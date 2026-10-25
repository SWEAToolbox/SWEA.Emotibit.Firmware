#include "SweaBleTransport.h"

#ifdef SWEA_BLE

#include <NimBLEDevice.h>
#include <Preferences.h>
#include <time.h>

// Optional fixed passkey for development builds (-DSWEA_BLE_PASSKEY=123456). Without it every device
// generates its own random 6-digit passkey on first boot and keeps it in NVS (see loadPasskey()).
#define SWEA_BLE_FW_STRING "swea-ble-0.7"
// Longest Control notification (opcode, status and arguments). Needs an MTU above 23, which the phone always asks for.
#define CONTROL_REPLY_MAX 64
// RAM for the record ring (newest data is kept when it is full; the SD card keeps everything).
#ifndef SWEA_BLE_RING_BYTES
#define SWEA_BLE_RING_BYTES 24576
#endif
// Sent notifications kept for RESUME_FROM (about 250 bytes each).
#ifndef SWEA_BLE_WINDOW_FRAMES
#define SWEA_BLE_WINDOW_FRAMES 32
#endif
#ifndef SWEA_BLE_MAX_BONDS
#define SWEA_BLE_MAX_BONDS 2
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
		OP_RESUME_FROM = 0x05,
		OP_ACK = 0x06,
		OP_SET_PROFILE = 0x07,
		OP_MARK_EVENT = 0x08,
		OP_RECORD = 0x09,
		OP_ENTER_PAIRING = 0x0A,
		OP_CLEAR_BONDS = 0x0B,
		OP_SLEEP = 0x0C
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
	uint32_t g_passkey = 123456;

	using swea::get16;
	using swea::put16;
	using swea::put32;

	// Connection parameters (protocol doc, section 1): interval 30-50 ms, no latency, supervision timeout 4 s.
	// A phone follows these (Windows did not): at 100-200 ms with latency 4 an Android phone carried only about
	// 2.5 notifications/s, less than the full profile produces, and the backlog starved the Control channel.
	const uint16_t CONN_MIN_INTERVAL = 24;	// units of 1.25 ms
	const uint16_t CONN_MAX_INTERVAL = 40;
	const uint16_t CONN_LATENCY = 0;
	const uint16_t CONN_TIMEOUT = 400;		// units of 10 ms
	// At most one data notification per this many ms (about 10 kB/s at full MTU, five times the full profile), so NimBLE's
	// buffers never fill up and Control replies and writes still get through.
	const uint32_t NOTIFY_SPACING_MS = 25;

	class ServerCallbacks : public NimBLEServerCallbacks
	{
		void onConnect(NimBLEServer *server, ble_gap_conn_desc *desc) override
		{
			// Only phones that are already bonded may connect, unless the pairing window is open.
			NimBLEAddress peer(desc->peer_id_addr);
			bool allowed = g_owner && (g_owner->isPairingOpen() || NimBLEDevice::isBonded(peer));
			if (!allowed)
			{
				Serial.println("SWEA BLE: rejected a connection from an unbonded phone (pairing window closed)");
				server->disconnect(desc->conn_handle);
				return;
			}
			g_owner->onConnected(desc->conn_handle);
			server->updateConnParams(desc->conn_handle, CONN_MIN_INTERVAL, CONN_MAX_INTERVAL, CONN_LATENCY, CONN_TIMEOUT);
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
		uint32_t onPassKeyRequest() override { return g_passkey; }
		void onAuthenticationComplete(ble_gap_conn_desc *desc) override
		{
			if (g_owner) g_owner->onEncryptionChange(desc->sec_state.encrypted);
			if (!desc->sec_state.encrypted)
			{
				NimBLEDevice::getServer()->disconnect(desc->conn_handle);	// failed pairing: drop the link
				return;
			}
			// Keep at most SWEA_BLE_MAX_BONDS bonded phones: forget the oldest ones, never the one connected now.
			NimBLEAddress current(desc->peer_id_addr);
			for (int guard = 0; guard < 8 && NimBLEDevice::getNumBonds() > SWEA_BLE_MAX_BONDS; guard++)
			{
				for (int i = 0; i < NimBLEDevice::getNumBonds(); i++)
				{
					NimBLEAddress a = NimBLEDevice::getBondedAddress(i);
					if (!(a == current)) { NimBLEDevice::deleteBond(a); break; }
				}
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

	if (_pairingOpen && (int32_t)(now - _pairingUntil) >= 0)
	{
		_pairingOpen = false;
		Serial.println("SWEA BLE: pairing window closed");
	}
	processCommands();
	drain();
	if (_connected && now - _lastStatusMs >= 1000)
	{
		_lastStatusMs = now;
		sendStatus(_encrypted);
	}
	return 0;
}

// EmotiBit's own control parser (parseIncomingControlPackets) reads text packets here. Commands that map onto EmotiBit
// features (recording, event markers, sleep) are turned into the same packets the Oscilloscope would send.
uint8_t SweaBleTransport::readControl(String &packet)
{
	if (_ctrlTail == _ctrlHead) return 0;
	packet = _ctrlOut[_ctrlTail];
	_ctrlOut[_ctrlTail] = "";
	_ctrlTail = (_ctrlTail + 1) % CTRL_OUT;
	return packet.length() > 255 ? 255 : (uint8_t)packet.length();
}

bool SweaBleTransport::pushControlPacket(const String &packet)
{
	uint8_t next = (_ctrlHead + 1) % CTRL_OUT;
	if (next == _ctrlTail) return false;	// queue full
	_ctrlOut[_ctrlHead] = packet;
	_ctrlHead = next;
	return true;
}

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

// Sends frames while a bonded phone is subscribed: first any requested replay, then new records from the ring.
// A few frames per call keep the main loop short.
void SweaBleTransport::drain()
{
	if (!_streaming || !_connected || !_encrypted || !_dataSubscribed) return;
	size_t cap = _mtu > 3 ? (size_t)_mtu - 3 : 20;
	if (cap > 244) cap = 244;
	uint8_t buf[244];

	if (_replay)
	{
		for (int frame = 0; frame < 6 && _replay; frame++)
		{
			if (millis() - _lastNotifyMs < NOTIFY_SPACING_MS) break;
			if (!swea::seqBefore(_replaySeq, _replayEnd)) { _replay = false; break; }
			size_t l = 0;
			if (_win.get(_replaySeq, buf, l))
			{
				if (!notifyData(buf, l)) { _notifyFailures++; return; }	// retry the same frame next time
				_lastNotifyMs = millis();
			}
			else _gap = true;											// fell out of the window while replaying
			_replaySeq++;
		}
		return;															// live data resumes after the replay
	}

	for (int frame = 0; frame < 6; frame++)
	{
		if (millis() - _lastNotifyMs < NOTIFY_SPACING_MS) break;
		swea::RecordRing::Cursor c = _ring.cursor();
		size_t n = swea::buildNotification(_ring, _seq, buf, cap, _profile);
		if (n == 0) break;
		if (!notifyData(buf, n))
		{
			_ring.restore(c);		// not sent: keep the records and try again later
			_notifyFailures++;
			break;
		}
		_win.store(_seq, buf, n);
		_seq++;
		_lastNotifyMs = millis();
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

void SweaBleTransport::openPairing(uint16_t seconds)
{
	if (!_on) return;
	_pairingUntil = millis() + (uint32_t)seconds * 1000UL;
	_pairingOpen = true;
	Serial.print("SWEA BLE: pairing open for "); Serial.print(seconds);
	Serial.print(" s. Passkey: "); Serial.println(_passkey);	// shown on Serial only (USB); also goes on the device label / QR code
}

// ---------------------------------------------------------------------------------------
// Per-device passkey: random 6 digits, generated on first boot and kept in NVS. A development build can force one.
void SweaBleTransport::loadPasskey()
{
#ifdef SWEA_BLE_PASSKEY
	_passkey = SWEA_BLE_PASSKEY;
	Serial.println("WARNING: fixed development passkey in use");
#else
	uint32_t pk = 0;
	Preferences prefs;
	if (prefs.begin("swea_ble", false))
	{
		pk = prefs.getUInt("passkey", 0);
		if (pk < 100000 || pk > 999999)
		{
			pk = 100000 + (esp_random() % 900000);
			prefs.putUInt("passkey", pk);
			Serial.println("SWEA BLE: generated a new device passkey");
		}
		prefs.end();
	}
	if (pk < 100000 || pk > 999999)
	{
		pk = 123456;
		Serial.println("WARNING: NVS unavailable, falling back to a fixed passkey");
	}
	_passkey = pk;
#endif
	g_passkey = _passkey;
}

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
	if (!_winMem)
	{
		_winMem = (uint8_t *)malloc(swea::FrameWindow::bytesFor(SWEA_BLE_WINDOW_FRAMES));
		if (_winMem) _win.init(_winMem, SWEA_BLE_WINDOW_FRAMES);
		else Serial.println("WARNING: no memory for the resume window; RESUME_FROM can only report a gap");
	}
	_replay = false; _gap = false;
	_cmdHead = _cmdTail = 0;
	_ctrlHead = _ctrlTail = 0; _recording = false;
	loadPasskey();

	String suffix = _deviceId.length() >= 4 ? _deviceId.substring(_deviceId.length() - 4) : String("0000");
	String name = "EmotiBit-" + suffix;

	NimBLEDevice::init(name.c_str());
	NimBLEDevice::setMTU(247);
	NimBLEDevice::setSecurityAuth(true, true, true);			// bonding, MITM protection, LE Secure Connections
	NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);	// passkey is shown on the label / QR code
	NimBLEDevice::setSecurityPasskey(_passkey);
#ifdef SWEA_BLE_PRIVACY
	// Resolvable private address so the phone cannot be tracked by a fixed address. Needs hardware verification.
	NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_RPA_PUBLIC_DEFAULT);
#endif

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
	if (NimBLEDevice::getNumBonds() == 0) openPairing(120);	// first use: no phone is bonded yet
}

void SweaBleTransport::stopBle()
{
	_on = false;
	_pairingOpen = false;
	_connected = _encrypted = _streaming = false;
	NimBLEDevice::deinit(true);
	if (_ringMem) { free(_ringMem); _ringMem = nullptr; }
	if (_winMem) { free(_winMem); _winMem = nullptr; _win.init(nullptr, 0); }
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
	// flags: bit1 SD ok (the firmware does not start without a card), bit2 gap, bit3 low battery, bit4 pairing open.
	// bit0 recording: as commanded over BLE.
	s[2] = 0x02 | (_recording ? 0x01 : 0) | (_gap ? 0x04 : 0) | ((_batteryPct != 0xFF && _batteryPct < 20) ? 0x08 : 0) | (_pairingOpen ? 0x10 : 0);
	s[3] = _batteryPct;						// 0xFF = unknown
	s[4] = _profile;
	s[5] = g_bootCount;
	put16(&s[6], _seq);
	put16(&s[8], _win.count() ? _win.oldestSeq() : _seq);
	put16(&s[10], (uint16_t)_ring.dropped());
	put32(&s[12], millis());
	put16(&s[16], (uint16_t)(ESP.getFreeHeap() / 1024));
	g_status->setValue(s, sizeof(s));
	if (notify && _connected) g_status->notify();
}

void SweaBleTransport::sendControlReply(uint8_t opcode, uint8_t status, const uint8_t *args, size_t n)
{
	if (!g_control || !_connected) return;
	uint8_t buf[CONTROL_REPLY_MAX];
	if (n > sizeof(buf) - 2) n = sizeof(buf) - 2;
	buf[0] = 0x80 | opcode;
	buf[1] = status;
	if (n) memcpy(&buf[2], args, n);
	g_control->setValue(buf, n + 2);
	g_control->notify();
}

// ---- BLE callbacks (host task): flags only -------------------------------------------
void SweaBleTransport::onConnected(uint16_t connHandle) { _connHandle = connHandle; _connected = true; _encrypted = false; _dataSubscribed = false; _mtu = 23; }
void SweaBleTransport::onDisconnected() { _connected = false; _encrypted = false; _streaming = false; _dataSubscribed = false; _replay = false; _connHandle = 0xFFFF; }
void SweaBleTransport::onEncryptionChange(bool encrypted) { _encrypted = encrypted; }
void SweaBleTransport::onMtu(uint16_t mtu) { _mtu = mtu; }
void SweaBleTransport::onSubscribe(bool subscribed) { _dataSubscribed = subscribed; }

// A write to Control is queued and executed by update() in the main loop, so the ring, the window and
// the replay state are only ever touched from one task.
void SweaBleTransport::onControlWrite(const uint8_t *data, size_t len)
{
	if (len < 1) return;
	Command c;
	c.len = len > CMD_MAX_LEN ? 0xFF : (uint8_t)len;	// 0xFF marks an over-long (invalid) command
	memcpy(c.d, data, len > CMD_MAX_LEN ? CMD_MAX_LEN : len);
	portENTER_CRITICAL(&_cmdMux);
	uint8_t next = (_cmdHead + 1) % CMD_QUEUE;
	if (next != _cmdTail) { _cmds[_cmdHead] = c; _cmdHead = next; }	// a full queue drops the write
	portEXIT_CRITICAL(&_cmdMux);
}

void SweaBleTransport::processCommands()
{
	for (int i = 0; i < CMD_QUEUE; i++)
	{
		Command c;
		portENTER_CRITICAL(&_cmdMux);
		bool have = _cmdTail != _cmdHead;
		if (have) { c = _cmds[_cmdTail]; _cmdTail = (_cmdTail + 1) % CMD_QUEUE; }
		portEXIT_CRITICAL(&_cmdMux);
		if (!have) return;
		execute(c);
	}
}

void SweaBleTransport::execute(const Command &c)
{
	uint8_t op = c.d[0];
	size_t len = c.len;
	const uint8_t *data = c.d;
	if (!_encrypted)
	{
		sendControlReply(op, ST_NOT_ALLOWED, nullptr, 0);
		return;
	}
	if (c.len == 0xFF)
	{
		sendControlReply(op, ST_BAD_ARGS, nullptr, 0);
		return;
	}
	switch (op)
	{
	case OP_GET_INFO:
	{
		// The phone registers the device on the server by this id, so it is sent in full.
		uint8_t out[CONTROL_REPLY_MAX - 2];
		size_t n = swea::buildInfoReply(out, sizeof(out), PROTOCOL_VERSION, SWEA_BLE_FW_STRING, _deviceId.c_str());
		sendControlReply(op, ST_OK, out, n);
		break;
	}
	case OP_START_STREAM:
		if (len != 2 || data[1] > 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		_profile = data[1];
		_streaming = true;
		_gap = false;
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	case OP_STOP_STREAM:
		_streaming = false;
		_replay = false;
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
	case OP_RESUME_FROM:
	{
		if (len != 3) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		uint16_t req = get16(&data[1]);
		if (swea::seqBefore(_seq, req)) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }	// asks for frames that were never sent
		uint16_t oldest = _win.count() ? _win.oldestSeq() : _seq;
		bool gap = swea::seqBefore(req, oldest);
		uint16_t first = gap ? oldest : req;
		_gap = gap;
		_replaySeq = first;
		_replayEnd = _seq;
		_replay = first != _seq;
		uint8_t out[3];
		put16(out, first);
		out[2] = gap ? 1 : 0;
		sendControlReply(op, ST_OK, out, sizeof(out));
		break;
	}
	case OP_ACK:
		if (len != 3) break;			// no reply for ACK (protocol doc)
		_win.ack(get16(&data[1]));
		break;
	case OP_SET_PROFILE:
		if (len != 2 || data[1] > 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		_profile = data[1];
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	case OP_MARK_EVENT:
	{
		// A UN packet through EmotiBit's control path is logged on the SD card with a packet number and comes back
		// through sendData(), so the event reaches the phone in the data stream too. Reply: the device clock.
		if (len != 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		if (!pushControlPacket("0,0,1,UN,1,100," + String((unsigned)data[1]))) { sendControlReply(op, ST_BUSY, nullptr, 0); break; }
		uint8_t out[4];
		put32(out, millis());
		sendControlReply(op, ST_OK, out, sizeof(out));
		break;
	}
	case OP_RECORD:
	{
		// off: 09 00      on: 09 01 <utc seconds u32> <milliseconds u16>   (names the files like the Oscilloscope: YYYY-MM-DD_HH-MM-SS-mmm)
		if (len == 2 && data[1] == 0)
		{
			if (!pushControlPacket("0,0,0,RE,1,100")) { sendControlReply(op, ST_BUSY, nullptr, 0); break; }
			_recording = false;
			sendControlReply(op, ST_OK, nullptr, 0);
			break;
		}
		if (len != 8 || data[1] != 1) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		time_t t = (time_t)swea::get32(&data[2]);
		uint16_t ms = get16(&data[6]);
		struct tm tmv;
		gmtime_r(&t, &tmv);
		if (tmv.tm_year + 1900 < 2020 || ms > 999) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }	// a phone with a wrong clock would name files 1970
		char name[32];
		snprintf(name, sizeof(name), "%04d-%02d-%02d_%02d-%02d-%02d-%03u", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec, (unsigned)ms);
		if (!pushControlPacket(String("0,0,1,RB,1,100,") + name)) { sendControlReply(op, ST_BUSY, nullptr, 0); break; }
		_recording = true;
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	}
	case OP_SLEEP:
		// MH = hibernate, the same packet the Oscilloscope sends. EmotiBit flushes the SD card and powers down in update().
		if (!pushControlPacket("0,0,0,MH,1,100")) { sendControlReply(op, ST_BUSY, nullptr, 0); break; }
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	case OP_ENTER_PAIRING:
	{
		if (len != 2) { sendControlReply(op, ST_BAD_ARGS, nullptr, 0); break; }
		openPairing(data[1] == 0 ? 120 : data[1]);
		sendControlReply(op, ST_OK, nullptr, 0);
		break;
	}
	case OP_CLEAR_BONDS:
		sendControlReply(op, ST_OK, nullptr, 0);
		NimBLEDevice::deleteAllBonds();
		Serial.println("SWEA BLE: all bonds deleted");
		disconnect();
		break;
	default:
		sendControlReply(op, ST_UNSUPPORTED, nullptr, 0);
	}
}

#endif // SWEA_BLE
