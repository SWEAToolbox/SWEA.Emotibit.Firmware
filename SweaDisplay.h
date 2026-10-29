/**************************************************************************/
/*!
    @file     SweaDisplay.h

    SWEA: Adafruit 128x64 OLED FeatherWing (SH1107, I2C 0x3C), built with -DSWEA_OLED.
    One screen: EDA and "Time connected". Button A (GPIO 15) creates a tag; the whole screen then shows a check mark
    and "Tagg skapad" for 3 s. Buttons B (GPIO 32 = hibernate) and C (GPIO 14 = accelerometer INT1) are never touched.
    The state and text live in SweaDisplayModel.h (host-tested). Docs: oled-display.md.

    The display uses I2C port 0 (Wire) on the Feather's own pins 23/22. EmotiBit's sensors use TwoWire(1) on 27/13.
*/
/**************************************************************************/
#pragma once
#ifdef SWEA_OLED

#include <Arduino.h>
#include "SweaDisplayModel.h"

class SweaBleTransport;

class SweaDisplay
{
public:
	static const uint8_t PIN_SDA = 23, PIN_SCL = 22, PIN_BUTTON_A = 15, I2C_ADDR = 0x3C;

	/** Probes the display. Without a display on the bus everything else is a no-op, so the same firmware runs with and without the wing. */
	bool begin();
	bool present() const { return _present; }

	/** Call from loop(). eda points to new EDA samples (n >= 0). Redraws at most twice per second, and at once on a screen change. */
	void update(SweaBleTransport &transport, const float *eda, size_t n);

private:
	bool _present = false;
	bool _lastButton = true;			// pull-up: high = released
	swea::DisplayModel _model;
	swea::DisplayModel::Screen _drawnScreen = swea::DisplayModel::SCREEN_OFF;
	uint32_t _drawnMs = 0;
	void draw(uint32_t now);
};

#endif
