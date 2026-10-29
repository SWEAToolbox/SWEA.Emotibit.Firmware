// Host unit test for SweaDisplayModel.h. Build and run with run_test.bat (MSVC) or run_test.sh (g++).
#include <stdio.h>
#include <string.h>
#include "../../SweaDisplayModel.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

int main()
{
	using namespace swea;
	char b[32];

	// Formatting
	formatDuration(0, b, sizeof(b)); CHECK(strcmp(b, "0h 00m 00s") == 0);
	formatDuration((52u * 60 + 10) * 1000, b, sizeof(b)); CHECK(strcmp(b, "0h 52m 10s") == 0);
	formatDuration((26u * 3600 + 5 * 60 + 9) * 1000 + 999, b, sizeof(b)); CHECK(strcmp(b, "26h 05m 09s") == 0);
	formatEda(3.4249f, true, b, sizeof(b)); CHECK(strcmp(b, "3.42") == 0);
	formatEda(0.0f, false, b, sizeof(b)); CHECK(strcmp(b, "--") == 0);
	formatEda(NAN, true, b, sizeof(b)); CHECK(strcmp(b, "--") == 0);

	formatBattery(87, b, sizeof(b)); CHECK(strcmp(b, "87%") == 0);
	formatBattery(100, b, sizeof(b)); CHECK(strcmp(b, "100%") == 0);
	formatBattery(5, b, sizeof(b)); CHECK(strcmp(b, "5%") == 0);
	formatBattery(0xFF, b, sizeof(b)); CHECK(strcmp(b, "--%") == 0);
	formatBattery(120, b, sizeof(b)); CHECK(strcmp(b, "100%") == 0);

	// Screen state
	DisplayModel m; m.begin(1000);
	CHECK(m.screen(1000) == DisplayModel::SCREEN_MAIN);
	CHECK(m.screen(1000 + DisplayModel::DIM_MS - 1) == DisplayModel::SCREEN_MAIN);
	CHECK(m.screen(1000 + DisplayModel::DIM_MS) == DisplayModel::SCREEN_OFF);

	// A press on a dark screen only wakes it
	uint32_t t = 1000 + DisplayModel::DIM_MS + 100;
	CHECK(!m.onButton(t));
	CHECK(m.screen(t) == DisplayModel::SCREEN_MAIN);
	CHECK(m.tagCount() == 0);

	// The next press creates a tag; the result starts the 3 s confirmation
	t += 500;
	CHECK(m.onButton(t));
	CHECK(m.nextTagCode() == 1);
	m.tagResult(true, t);
	CHECK(m.tagCount() == 1);
	CHECK(m.screen(t) == DisplayModel::SCREEN_TAG_OK);
	CHECK(m.screen(t + 2999) == DisplayModel::SCREEN_TAG_OK);
	CHECK(m.screen(t + 3000) == DisplayModel::SCREEN_MAIN);

	// Presses during the confirmation are ignored (no second tag)
	CHECK(!m.onButton(t + 1000));
	CHECK(m.tagCount() == 1);
	CHECK(m.screen(t + 1000) == DisplayModel::SCREEN_TAG_OK);

	// After the confirmation the screen stays on for 30 s from the tag
	CHECK(m.screen(t + 3000 + 100) == DisplayModel::SCREEN_MAIN);
	CHECK(m.nextTagCode() == 2);

	// Bounce within 30 ms is ignored
	t += 5000;
	CHECK(m.onButton(t));
	CHECK(!m.onButton(t + 10));

	// Failure path: no count, failure screen
	m.tagResult(false, t + 100);
	CHECK(m.tagCount() == 1);
	CHECK(m.screen(t + 100) == DisplayModel::SCREEN_TAG_FAIL);

	// Tag code wraps 255 -> 1 and is never 0
	DisplayModel w; w.begin(0);
	uint32_t now = 100;
	for (int i = 0; i < 255; i++) { now += 1000; CHECK(w.onButton(now)); CHECK(w.nextTagCode() != 0); w.tagResult(true, now); now += DisplayModel::CONFIRM_MS; }
	CHECK(w.tagCount() == 255);
	CHECK(w.nextTagCode() == 1);

	// millis() wrap-around during the confirmation
	DisplayModel r; r.begin(0xFFFFF000u);
	r.tagResult(true, 0xFFFFFC00u);
	CHECK(r.screen(0xFFFFFFFFu) == DisplayModel::SCREEN_TAG_OK);
	CHECK(r.screen(0x00000100u) == DisplayModel::SCREEN_TAG_OK);
	CHECK(r.screen(0x00000C00u) == DisplayModel::SCREEN_MAIN);

	// Link and EDA values pass through
	r.setEda(2.5f); r.setLink(true, 4242);
	CHECK(r.edaValid() && r.eda() == 2.5f && r.connected() && r.connectedMs() == 4242);

	{ DisplayModel bm; CHECK(bm.battery() == 0xFF); bm.setBattery(42); CHECK(bm.battery() == 42); }

	printf("%d checks, %d failed\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
