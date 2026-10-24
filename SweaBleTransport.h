/**************************************************************************/
/*!
    @file     SweaBleTransport.h

    SWEA: BLE transport that replaces EmotiBitWiFi when built with -DSWEA_BLE.
    It exposes the subset of the EmotiBitWiFi API that EmotiBit.cpp uses, so
    the upstream sampling, SD logging and packet code is untouched.

    Protocol: docs/SWEA_BLE_PROTOCOL.md (version 1, draft).
    Wi-Fi is never initialised in this build.
*/
/**************************************************************************/
#pragma once
#ifdef SWEA_BLE

#include <Arduino.h>
#include "SweaBleCodec.h"

// EmotiBit.cpp compares status() against the Arduino Wi-Fi constant. No Wi-Fi header is
// included in this build, so provide the one value that is used.
#ifndef WL_CONNECTED
#define WL_CONNECTED 3
#endif
#ifndef WL_DISCONNECTED
#define WL_DISCONNECTED 6
#endif

class SweaBleTransport
{
public:
	static const uint8_t PROTOCOL_VERSION = 1;
	static const uint8_t MAX_CREDENTIALS = 12;	// kept only so shared code compiles

	// ---- API used by EmotiBit.cpp (same names as EmotiBitWiFi) -------------------------
	uint8_t begin(int32_t timeout = 61500, uint8_t maxAttemptsPerCred = 2, uint16_t attemptDelay = 3000);
	void end();
	int8_t disconnect();
	int8_t update(String &syncPackets, uint16_t &syncPacketCounter);
	uint8_t readControl(String &packet);
	int8_t sendData(const String &message);
	int8_t addCredential(const String &ssid, const String &userid, const String &username, const String &password);
	void setDeviceId(const String emotibitDeviceId);
	bool isConnected();
	bool isOff();
	void updateStatus();
	uint8_t status(bool update = true);
	uint8_t getNumCredentials();
	bool isEnterpriseNetworkListed();
	static uint8_t getMaxNumCredentialAllowed();

	// ---- SWEA specific ---------------------------------------------------------------
	bool isStreaming() const { return _streaming; }

	// Called from the BLE callbacks (NimBLE host task)
	void onConnected(uint16_t connHandle);
	void onDisconnected();
	void onEncryptionChange(bool encrypted);
	void onMtu(uint16_t mtu);
	void onSubscribe(bool subscribed);
	void onControlWrite(const uint8_t *data, size_t len);

private:
	String _deviceId = "";
	volatile bool _on = false;
	volatile bool _connected = false;
	volatile bool _encrypted = false;
	volatile bool _streaming = false;
	volatile uint16_t _mtu = 23;
	volatile uint16_t _connHandle = 0xFFFF;
	volatile bool _dataSubscribed = false;
	uint16_t _seq = 0;
	uint32_t _lastStatusMs = 0;
	uint8_t _profile = 0;
	uint8_t _batteryPct = 0xFF;			// 0xFF = unknown until a B% record has been seen
	swea::RecordRing _ring;
	uint8_t *_ringMem = nullptr;
	swea::ParsedPacket _pp;
	uint32_t _notifyFailures = 0;

	void handleLine(const char *s, size_t len);
	void drain();

	void startBle();
	void stopBle();
	bool notifyData(const uint8_t *buf, size_t len);
	void sendStatus(bool notify);
	void sendControlReply(uint8_t opcode, uint8_t status, const uint8_t *args, size_t n);
};

#endif // SWEA_BLE
