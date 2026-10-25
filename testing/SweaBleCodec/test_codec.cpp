// Host unit test for SweaBleCodec.h. Build and run with run_test.bat (MSVC) or run_test.sh (g++).
#include <stdio.h>
#include <string.h>
#include <vector>
#include "../../SweaBleCodec.h"

static int g_fail = 0, g_checks = 0;
#define CHECK(c) do { g_checks++; if (!(c)) { g_fail++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

static bool parse(const char *s, swea::ParsedPacket &p) { return swea::parsePacketLine(s, strlen(s), p); }

int main()
{
	using namespace swea;

	// 1. Protocol document test vector (docs/SWEA_BLE_PROTOCOL.md section 8).
	{
		static uint8_t mem[256]; RecordRing ring; ring.init(mem, sizeof(mem));
		float v[2] = { 0.030269f, 0.030269f };
		CHECK(ring.push("EA", 2, 531459, v));
		uint8_t out[244];
		size_t n = buildNotification(ring, 7, out, sizeof(out));
		const uint8_t expect[] = { 0x01,0x07,0x00, 0x45,0x41,0x02,0x03,0x1c,0x08,0x00, 0xb2,0xf6,0xf7,0x3c, 0xb2,0xf6,0xf7,0x3c };
		CHECK(n == sizeof(expect));
		CHECK(n == sizeof(expect) && memcmp(out, expect, n) == 0);
		CHECK(ring.empty());
	}

	// 2. Parsing SD-card lines.
	{
		ParsedPacket p;
		CHECK(parse("531459,17301,2,EA,1,100,0.030269,0.030269", p));
		CHECK(p.tag[0] == 'E' && p.tag[1] == 'A' && p.n == 2 && p.t_ms == 531459);
		CHECK(p.v[0] > 0.03026f && p.v[0] < 0.03028f);
		CHECK(parse("531473,17298,3,PI,1,100,112870,112866,112867", p));
		CHECK(p.n == 3 && p.v[0] == 112870.0f && p.v[2] == 112867.0f);
		CHECK(parse("531459,17302,2,EL,1,100,26425.000000,26425.000000", p) && p.v[0] == 26425.0f);
		CHECK(parse("1000,1,1,B%,1,100,87", p) && p.v[0] == 87.0f);
		CHECK(parse("1000,1,3,AX,1,100,-0.436,0.1,-9.8", p) && p.v[0] < -0.43f && p.v[2] < -9.7f);
		CHECK(!parse("531473,5,1,EM,1,100,MN", p));		// mode packet, not a data tag
		CHECK(!parse("531473,5,1,AK,1,100,12", p));		// control
		CHECK(!parse("abc,1,1,EA,1,100,1", p));			// bad timestamp
		CHECK(!parse("1000,1,1,EA,1,100", p));			// no payload
		CHECK(!parse("1000,1,1,EA,1,100,12x", p));		// non numeric payload
		CHECK(!parse("", p));
		CHECK(!parse("1000,1", p));
		CHECK(!parse("1000,1,1,EAA,1,100,1", p));		// tag length 3
	}

	// 3. Profiles.
	CHECK(profileAllows(0, "PI") && profileAllows(0, "MX") && profileAllows(2, "PG"));
	CHECK(!profileAllows(1, "PI") && !profileAllows(1, "PG") && !profileAllows(1, "MZ"));
	CHECK(profileAllows(1, "EA") && profileAllows(1, "AX") && profileAllows(1, "T1") && profileAllows(1, "HR"));

	// 4. Splitting a long record over small frames; values keep order and t_ms.
	{
		static uint8_t mem[512]; RecordRing ring; ring.init(mem, sizeof(mem));
		float v[10]; for (int i = 0; i < 10; i++) v[i] = (float)(i + 1);
		CHECK(ring.push("PG", 10, 4242, v));
		std::vector<float> got;
		uint8_t out[3 + 7 + 4 * 4];	// room for 4 floats
		int frames = 0; uint16_t seq = 0;
		while (true)
		{
			size_t n = buildNotification(ring, seq++, out, sizeof(out));
			if (!n) break;
			frames++;
			CHECK(out[0] == 1);
			CHECK(out[3] == 'P' && out[4] == 'G');
			CHECK(get32(&out[6]) == 4242);
			for (int i = 0; i < out[5]; i++) { float f; memcpy(&f, &out[10 + 4 * i], 4); got.push_back(f); }
		}
		CHECK(frames == 3 && got.size() == 10);
		for (int i = 0; i < (int)got.size() && i < 10; i++) CHECK(got[i] == (float)(i + 1));
		CHECK(ring.empty());
	}

	// 5. Several records share one frame; a record that does not fit starts the next frame.
	{
		static uint8_t mem[512]; RecordRing ring; ring.init(mem, sizeof(mem));
		float a[2] = { 1, 2 }, b[2] = { 3, 4 };
		ring.push("EA", 2, 1, a); ring.push("T1", 2, 2, b); ring.push("HR", 1, 3, a);
		uint8_t out[3 + 15 + 15 + 11];
		size_t n = buildNotification(ring, 1, out, sizeof(out));
		CHECK(n == 3 + 15 + 15 + 11);
		CHECK(ring.empty());
	}

	// 6. A failed send is rolled back and the same bytes are produced again.
	{
		static uint8_t mem[256]; RecordRing ring; ring.init(mem, sizeof(mem));
		float v[3] = { 1, 2, 3 };
		ring.push("AX", 3, 10, v); ring.push("AY", 3, 10, v);
		RecordRing::Cursor c = ring.cursor();
		uint8_t o1[100], o2[100];
		size_t n1 = buildNotification(ring, 5, o1, 3 + 19);	// fits one 3-value record
		ring.restore(c);
		size_t n2 = buildNotification(ring, 5, o2, 3 + 19);
		CHECK(n1 == n2 && n1 == 22 && memcmp(o1, o2, n1) == 0);
		CHECK(!ring.empty());
	}

	// 7. Full ring drops the oldest records, keeps the newest, and counts the drops. Wrap-around works.
	{
		static uint8_t mem[100]; RecordRing ring; ring.init(mem, sizeof(mem));
		for (uint32_t t = 1; t <= 20; t++) { float v[2] = { (float)t, 0 }; CHECK(ring.push("EA", 2, t, v)); }	// 17 bytes each
		CHECK(ring.dropped() > 0 && ring.used() <= sizeof(mem));
		RecordRing::View r; uint32_t last = 0; int count = 0;
		while (ring.peek(r)) { CHECK(r.t_ms > last); last = r.t_ms; ring.popHead(); count++; }
		CHECK(last == 20);
		CHECK((uint32_t)count + ring.dropped() == 20);
	}

	// 8. Round trip: SD text line -> parse -> ring -> notification -> decode.
	{
		static uint8_t mem[512]; RecordRing ring; ring.init(mem, sizeof(mem));
		ParsedPacket p;
		CHECK(parse("531473,17298,3,PI,1,100,112870,112866,112867", p));
		CHECK(ring.push(p.tag, p.n, p.t_ms, p.v));
		uint8_t out[244];
		size_t n = buildNotification(ring, 0, out, sizeof(out));
		CHECK(n == 3 + 7 + 12);
		CHECK(out[3] == 'P' && out[4] == 'I' && out[5] == 3 && get32(&out[6]) == 531473);
		float f; memcpy(&f, &out[10], 4); CHECK(f == 112870.0f);
	}

	// 9. Too small a frame produces nothing instead of looping.
	{
		static uint8_t mem[64]; RecordRing ring; ring.init(mem, sizeof(mem));
		float v[1] = { 1 }; ring.push("EA", 1, 1, v);
		uint8_t out[8];
		CHECK(buildNotification(ring, 0, out, sizeof(out)) == 0);
	}


	// 10. FrameWindow: store, get, overwrite oldest, ack, sequence wrap-around.
	{
		static uint8_t mem[FrameWindow::bytesFor(4)];
		FrameWindow w; w.init(mem, 4);
		uint8_t f[10], o[FrameWindow::MAX_FRAME]; size_t l;
		CHECK(w.count() == 0);
		for (uint16_t seq = 65533; seq != (uint16_t)(65533 + 6); seq++)	// crosses the 16-bit wrap
		{
			memset(f, (uint8_t)seq, sizeof(f));
			CHECK(w.store(seq, f, sizeof(f)));
		}
		CHECK(w.count() == 4);								// 6 stored, 4 slots
		CHECK(w.nextSeq() == (uint16_t)(65533 + 6));
		CHECK(w.oldestSeq() == (uint16_t)(65533 + 2));
		CHECK(!w.get(65533, o, l) && !w.get(65534, o, l));	// overwritten
		CHECK(w.get(65535, o, l) && l == 10 && o[0] == 0xFF);
		CHECK(w.get(0, o, l) && o[0] == 0);				// after the wrap
		CHECK(w.get(2, o, l) && o[0] == 2);
		CHECK(!w.get(3, o, l));							// not sent yet
		w.ack(0);											// frees 65535 and 0
		CHECK(w.count() == 2 && w.oldestSeq() == 1);
		w.ack(1000);										// acking beyond the newest empties the window
		CHECK(w.count() == 0);
		CHECK(seqBefore(65535, 1) && !seqBefore(1, 65535) && !seqBefore(5, 5));
	}

	printf("%d checks, %d failed\n", g_checks, g_fail);
	return g_fail ? 1 : 0;
}
