/**************************************************************************/
/*!
    @file     SweaDisplayModel.h

    SWEA: pure C++ state and text formatting for the OLED FeatherWing (docs: oled-display.md).
    One screen: EDA, battery, "Time connected", and a 3 s full-screen confirmation after button A creates a tag.
    No Arduino or display dependencies, so it is unit-tested on a host (testing/SweaDisplayModel).
*/
/**************************************************************************/
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>

namespace swea
{
	/** "0h 52m 10s" (hours are not capped). */
	inline void formatDuration(uint32_t ms, char *out, size_t cap)
	{
		uint32_t s = ms / 1000;
		snprintf(out, cap, "%luh %02lum %02lus", (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60));
	}

	/** EDA in microsiemens with 2 decimals ("3.42"); "--" while there is no value yet. */
	inline void formatEda(float uS, bool valid, char *out, size_t cap)
	{
		if (!valid || isnan(uS)) snprintf(out, cap, "--");
		else snprintf(out, cap, "%.2f", (double)uS);
	}

	/** Battery as "87%"; "--%" while unknown (0xFF, no B% record seen yet). */
	inline void formatBattery(uint8_t pct, char *out, size_t cap)
	{
		if (pct == 0xFF) snprintf(out, cap, "--%%");
		else snprintf(out, cap, "%u%%", (unsigned)(pct > 100 ? 100 : pct));
	}

	class DisplayModel
	{
	public:
		static const uint32_t CONFIRM_MS = 3000;	// full-screen "Tagg skapad"
		static const uint32_t DIM_MS = 30000;		// screen off after this long without activity
		static const uint32_t DEBOUNCE_MS = 30;

		enum Screen : uint8_t { SCREEN_OFF, SCREEN_MAIN, SCREEN_TAG_OK, SCREEN_TAG_FAIL };

		void begin(uint32_t now) { _activityMs = now; _lastPressMs = now - DEBOUNCE_MS; }

		void setEda(float uS) { _eda = uS; _edaValid = true; }
		bool edaValid() const { return _edaValid; }
		float eda() const { return _eda; }

		void setBattery(uint8_t pct) { _battery = pct; }
		uint8_t battery() const { return _battery; }

		/** connectedMs = time since the link was established, 0 when not connected. */
		void setLink(bool connected, uint32_t connectedMs) { _connected = connected; _connectedMs = connectedMs; }
		bool connected() const { return _connected; }
		uint32_t connectedMs() const { return _connectedMs; }

		/** Button A pressed (falling edge). Returns true if a tag must be created now. The first press on a dark screen
		 *  only wakes it. Presses during the confirmation and bounces are ignored. */
		bool onButton(uint32_t now)
		{
			if (now - _lastPressMs < DEBOUNCE_MS) return false;
			_lastPressMs = now;
			bool wasOff = screen(now) == SCREEN_OFF;
			_activityMs = now;
			if (wasOff) return false;
			if (_resultUntilMs != 0 && (int32_t)(_resultUntilMs - now) > 0) return false;
			return true;
		}

		/** Report whether the tag was created; starts the 3 s confirmation. */
		void tagResult(bool ok, uint32_t now)
		{
			if (ok) _tagCount++;
			_resultOk = ok;
			_resultUntilMs = now + CONFIRM_MS;
			if (_resultUntilMs == 0) _resultUntilMs = 1;	// 0 means "none"
			_activityMs = now;
		}

		uint8_t tagCount() const { return _tagCount; }

		Screen screen(uint32_t now) const
		{
			if (_resultUntilMs != 0 && (int32_t)(_resultUntilMs - now) > 0) return _resultOk ? SCREEN_TAG_OK : SCREEN_TAG_FAIL;
			if (now - _activityMs >= DIM_MS) return SCREEN_OFF;
			return SCREEN_MAIN;
		}

		/** Wake the screen without a tag (pairing window, errors). */
		void wake(uint32_t now) { _activityMs = now; }

		/** Tag code sent to the server: running number 1..255, then wraps to 1. */
		uint8_t nextTagCode() const { return (uint8_t)((_tagCount % 255) + 1); }

	private:
		float _eda = 0;
		bool _edaValid = false;
		uint8_t _battery = 0xFF;
		bool _connected = false;
		uint32_t _connectedMs = 0;
		uint32_t _activityMs = 0;
		uint32_t _lastPressMs = 0;
		uint32_t _resultUntilMs = 0;
		bool _resultOk = false;
		uint8_t _tagCount = 0;
	};
}
