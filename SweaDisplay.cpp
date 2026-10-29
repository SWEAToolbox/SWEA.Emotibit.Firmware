#ifdef SWEA_OLED

#include "SweaDisplay.h"
#include "SweaBleTransport.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

using swea::DisplayModel;

static Adafruit_SH1107 g_oled(64, 128, &Wire);

bool SweaDisplay::begin()
{
	Wire.begin(PIN_SDA, PIN_SCL);
	Wire.setClock(400000);
	// Diagnostics on Serial (one line, no secrets): every address that answers on the display bus.
	String found = "";
	for (uint8_t a = 1; a < 127; a++)
	{
		Wire.beginTransmission(a);
		if (Wire.endTransmission() == 0) found += String(" 0x") + String(a, HEX);
	}
	Serial.println("OLED: I2C devices on SDA23/SCL22:" + (found.length() ? found : String(" none")));
	Wire.beginTransmission(I2C_ADDR);
	if (Wire.endTransmission() != 0) { Serial.println("OLED: no display at 0x3C, display off"); _present = false; return false; }
	if (!g_oled.begin(I2C_ADDR, true)) { Serial.println("OLED: SH1107 begin() failed"); _present = false; return false; }
	g_oled.setRotation(1);			// 128x64 landscape, as in Adafruit's SH1107 FeatherWing example
	g_oled.clearDisplay();
	g_oled.display();
	pinMode(PIN_BUTTON_A, INPUT_PULLUP);
	_model.begin(millis());
	_present = true;
	Serial.println("OLED: display ready");
	return true;
}

void SweaDisplay::update(SweaBleTransport &transport, const float *eda, size_t n)
{
	if (!_present) return;
	uint32_t now = millis();

	if (n > 0) _model.setEda(eda[n - 1]);
	uint32_t cms = transport.connectedMs();
	_model.setLink(cms != 0, cms);
	_model.setBattery(transport.batteryPct());

	bool button = digitalRead(PIN_BUTTON_A);
	if (!button && _lastButton)
	{
		if (_model.onButton(now))
		{
			bool ok = transport.markEvent(_model.nextTagCode());
			_model.tagResult(ok, now);
		}
	}
	_lastButton = button;

	DisplayModel::Screen s = _model.screen(now);
	bool changed = s != _drawnScreen;
	if (!changed && (s == DisplayModel::SCREEN_OFF || now - _drawnMs < 500)) return;	// <= 2 Hz
	_drawnScreen = s;
	_drawnMs = now;
	draw(now);
}

void SweaDisplay::draw(uint32_t now)
{
	DisplayModel::Screen s = _model.screen(now);
	g_oled.clearDisplay();
	g_oled.setTextColor(SH110X_WHITE);

	if (s == DisplayModel::SCREEN_TAG_OK)
	{
		// check mark on the left (thick polyline), text on the right (size 2 fits 10 characters per 128 px, so two lines)
		for (int8_t d = -2; d <= 2; d++)
		{
			g_oled.drawLine(2, 32 + d, 20, 50 + d, SH110X_WHITE);
			g_oled.drawLine(20, 50 + d, 50, 12 + d, SH110X_WHITE);
		}
		g_oled.setTextSize(2);
		g_oled.setCursor(54, 16);
		g_oled.print("Tagg");
		g_oled.setCursor(54, 36);
		g_oled.print("skapad");
	}
	else if (s == DisplayModel::SCREEN_TAG_FAIL)
	{
		for (int8_t d = -1; d <= 1; d++)
		{
			g_oled.drawLine(48, 8 + d, 80, 36 + d, SH110X_WHITE);
			g_oled.drawLine(80, 8 + d, 48, 36 + d, SH110X_WHITE);
		}
		g_oled.setTextSize(1);
		g_oled.setCursor(16, 50);
		g_oled.print("Tagg misslyckades");
	}
	else if (s == DisplayModel::SCREEN_MAIN)
	{
		char buf[24];
		g_oled.setTextSize(1);
		g_oled.setCursor(0, 0);
		g_oled.print("EDA (uS)");
		// battery, top right, size 2 (up to 4 characters = 48 px)
		swea::formatBattery(_model.battery(), buf, sizeof(buf));
		g_oled.setTextSize(2);
		g_oled.setCursor(128 - 12 * (int16_t)strlen(buf), 0);
		g_oled.print(buf);
		swea::formatEda(_model.eda(), _model.edaValid(), buf, sizeof(buf));
		g_oled.setTextSize(3);
		g_oled.setCursor(0, 16);
		g_oled.print(buf);
		g_oled.setTextSize(1);
		g_oled.setCursor(0, 41);
		g_oled.print("Time connected");
		g_oled.setTextSize(2);
		g_oled.setCursor(0, 49);
		if (_model.connected())
		{
			swea::formatDuration(_model.connectedMs(), buf, sizeof(buf));
			g_oled.print(buf);
		}
		else g_oled.print("Ej ansluten");
	}
	g_oled.display();	// SCREEN_OFF: cleared frame
}

#endif
