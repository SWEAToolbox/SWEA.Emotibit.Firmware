/**************************************************************************/
/*!
    @file     SweaBleCodec.h

    SWEA: pure C++ pieces of the BLE protocol (docs/SWEA_BLE_PROTOCOL.md):
      - parsing EmotiBit text packets (the lines written to the SD card),
      - the in-RAM record ring buffer,
      - building data notifications.
    No Arduino or ESP dependencies, so it is unit-tested on a host
    (testing/SweaBleCodec).
*/
/**************************************************************************/
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

namespace swea
{
	static const uint8_t PROTOCOL_VERSION = 1;
	static const size_t NOTIFICATION_HEADER_LEN = 3;	// ver:u8 seq:u16
	static const size_t RECORD_HEADER_LEN = 7;			// tag[2] n:u8 t_ms:u32
	static const size_t MAX_VALUES = 128;				// values per record held in RAM

	inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
	inline void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)((v >> 8) & 0xFF); p[2] = (uint8_t)((v >> 16) & 0xFF); p[3] = (uint8_t)(v >> 24); }
	inline uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
	inline uint32_t get32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }

	// Tags that carry numeric sample data. Everything else (control, mode, info, sync packets) is not forwarded.
	inline bool isDataTag(const char *t)
	{
		static const char *const tags[] = {
			"EA","EL","ER","PI","PR","PG","O2","T0","T1","TH","H0",
			"AX","AY","AZ","GX","GY","GZ","MX","MY","MZ",
			"BV","B%","SD","HR","BI","SA","SR","SF","DC","DO" };
		for (size_t i = 0; i < sizeof(tags) / sizeof(tags[0]); i++)
			if (t[0] == tags[i][0] && t[1] == tags[i][1]) return true;
		return false;
	}

	// Profile 0 = all channels, 1 = EDA + temperature + motion (no PPG, no magnetometer), 2 = all (100 Hz PPG build).
	inline bool profileAllows(uint8_t profile, const char *t)
	{
		if (profile != 1) return true;
		if (t[0] == 'P' && (t[1] == 'I' || t[1] == 'R' || t[1] == 'G')) return false;	// PPG
		if (t[0] == 'M' && (t[1] == 'X' || t[1] == 'Y' || t[1] == 'Z')) return false;	// magnetometer
		return true;
	}

	struct ParsedPacket
	{
		char tag[2];
		uint8_t n;
		uint32_t t_ms;
		float v[MAX_VALUES];
	};

	/** Parse one EmotiBit text packet line: "TIMESTAMP,PACKET#,COUNT,TAG,VERSION,RELIABILITY,v0,v1,...".
	 *  Returns true only for a well formed packet with a data tag and at least one numeric value. */
	inline bool parsePacketLine(const char *s, size_t len, ParsedPacket &out)
	{
		const char *p = s, *end = s + len;
		auto nextField = [&](const char *&f, const char *&fe) -> bool {
			if (p > end) return false;
			f = p;
			while (p < end && *p != ',') p++;
			fe = p;
			if (p < end) p++; else p = end + 1;	// consumed the last field
			return true;
		};
		const char *f, *fe;
		char tmp[24];
		auto toNum = [&](const char *a, const char *b, char *dst) -> bool {
			size_t l = (size_t)(b - a);
			if (l == 0 || l >= 24) return false;
			memcpy(dst, a, l); dst[l] = 0; return true;
		};
		if (!nextField(f, fe) || !toNum(f, fe, tmp)) return false;
		char *ep;
		unsigned long ts = strtoul(tmp, &ep, 10);
		if (*ep) return false;
		if (!nextField(f, fe)) return false;			// packet number (not forwarded)
		if (!nextField(f, fe) || !toNum(f, fe, tmp)) return false;
		unsigned long count = strtoul(tmp, &ep, 10);	// declared value count
		if (*ep) return false;
		if (!nextField(f, fe) || (fe - f) != 2) return false;
		out.tag[0] = f[0]; out.tag[1] = f[1];
		if (!isDataTag(out.tag)) return false;
		if (!nextField(f, fe)) return false;			// protocol version
		if (!nextField(f, fe)) return false;			// reliability
		size_t n = 0;
		while (p <= end && n < MAX_VALUES)
		{
			if (!nextField(f, fe)) break;
			if (!toNum(f, fe, tmp)) return false;
			float v = strtof(tmp, &ep);
			if (*ep) return false;
			out.v[n++] = v;
		}
		if (n == 0 || (count != 0 && n > count)) return false;
		out.n = (uint8_t)n;
		out.t_ms = (uint32_t)ts;
		return true;
	}

	/** Byte ring of records. Each stored record is [len:u16][tag:2][n:u8][t_ms:u32][f32 x n].
	 *  When full, the oldest records are dropped (and counted) so the newest data is kept. */
	class RecordRing
	{
	public:
		struct View
		{
			char tag[2];
			uint8_t n;
			uint32_t t_ms;
			float v[MAX_VALUES];
		};
		struct Cursor { size_t head; size_t used; uint8_t partial; };

		void init(uint8_t *buf, size_t cap) { _b = buf; _cap = cap; _head = 0; _used = 0; _partial = 0; _dropped = 0; }
		size_t used() const { return _used; }
		size_t capacity() const { return _cap; }
		uint32_t dropped() const { return _dropped; }
		bool empty() const { return _used == 0; }

		bool push(const char *tag, uint8_t n, uint32_t t_ms, const float *v)
		{
			if (!_b || n == 0) return false;
			size_t body = RECORD_HEADER_LEN + 4u * n;
			size_t need = 2 + body;
			if (need > _cap) return false;
			while (_cap - _used < need) dropHead();
			uint8_t tmp[2 + RECORD_HEADER_LEN + 4 * MAX_VALUES];
			put16(tmp, (uint16_t)body);
			tmp[2] = (uint8_t)tag[0]; tmp[3] = (uint8_t)tag[1]; tmp[4] = n;
			put32(&tmp[5], t_ms);
			for (uint8_t i = 0; i < n; i++) memcpy(&tmp[9 + 4 * i], &v[i], 4);
			size_t tail = (_head + _used) % _cap;
			for (size_t i = 0; i < need; i++) _b[(tail + i) % _cap] = tmp[i];
			_used += need;
			return true;
		}

		bool peek(View &out) const
		{
			if (_used == 0) return false;
			uint8_t hdr[2];
			rd(_head, hdr, 2);
			size_t body = get16(hdr);
			uint8_t rec[RECORD_HEADER_LEN + 4 * MAX_VALUES];
			rd((_head + 2) % _cap, rec, body);
			out.tag[0] = (char)rec[0]; out.tag[1] = (char)rec[1];
			out.n = rec[2];
			out.t_ms = get32(&rec[3]);
			for (uint8_t i = 0; i < out.n; i++) memcpy(&out.v[i], &rec[RECORD_HEADER_LEN + 4 * i], 4);
			return true;
		}

		void popHead() { if (_used) { dropHeadInternal(); } }
		uint8_t partial() const { return _partial; }
		void setPartial(uint8_t k) { _partial = k; }
		Cursor cursor() const { Cursor c = { _head, _used, _partial }; return c; }
		void restore(const Cursor &c) { _head = c.head; _used = c.used; _partial = c.partial; }

	private:
		uint8_t *_b = nullptr;
		size_t _cap = 0, _head = 0, _used = 0;
		uint8_t _partial = 0;	// floats of the head record already sent
		uint32_t _dropped = 0;

		void rd(size_t pos, uint8_t *dst, size_t n) const { for (size_t i = 0; i < n; i++) dst[i] = _b[(pos + i) % _cap]; }
		void dropHeadInternal()
		{
			uint8_t hdr[2];
			rd(_head, hdr, 2);
			size_t total = 2 + (size_t)get16(hdr);
			_head = (_head + total) % _cap;
			_used -= total;
			_partial = 0;
		}
		void dropHead() { dropHeadInternal(); _dropped++; }
	};

	/** Window of the most recently sent notifications, for RESUME_FROM replay. Fixed slots, oldest overwritten.
	 *  Sequence numbers are compared modulo 2^16. */
	class FrameWindow
	{
	public:
		static const size_t MAX_FRAME = 244;
		static const size_t SLOT_BYTES = 2 + 1 + MAX_FRAME;	// seq:u16 len:u8 data

		void init(uint8_t *mem, size_t slots) { _m = mem; _slots = slots; _head = 0; _count = 0; _next = 0; }
		static constexpr size_t bytesFor(size_t slots) { return slots * SLOT_BYTES; }
		size_t count() const { return _count; }
		uint16_t nextSeq() const { return _next; }	// sequence number after the newest stored frame
		uint16_t oldestSeq() const { return (uint16_t)(_next - _count); }

		/** Store a frame that was just sent. Frames must be stored with consecutive sequence numbers. */
		bool store(uint16_t seq, const uint8_t *data, size_t len)
		{
			if (!_m || len == 0 || len > MAX_FRAME) return false;
			if (_count == _slots) { _head = (_head + 1) % _slots; _count--; }
			size_t idx = (_head + _count) % _slots;
			uint8_t *slot = _m + idx * SLOT_BYTES;
			put16(slot, seq); slot[2] = (uint8_t)len; memcpy(slot + 3, data, len);
			_count++;
			_next = (uint16_t)(seq + 1);
			return true;
		}

		/** Copy the frame with this sequence number. False if it is not (or no longer) in the window. */
		bool get(uint16_t seq, uint8_t *out, size_t &len) const
		{
			uint16_t off = (uint16_t)(seq - oldestSeq());
			if (off >= _count) return false;
			const uint8_t *slot = _m + ((_head + off) % _slots) * SLOT_BYTES;
			len = slot[2];
			memcpy(out, slot + 3, len);
			return true;
		}

		/** The phone has received everything up to and including seq: free those slots. */
		void ack(uint16_t seq)
		{
			while (_count > 0 && (int16_t)(oldestSeq() - seq) <= 0) { _head = (_head + 1) % _slots; _count--; }
		}

	private:
		uint8_t *_m = nullptr;
		size_t _slots = 0, _head = 0, _count = 0;
		uint16_t _next = 0;
	};

	/** True if sequence a is before b (modulo 2^16). */
	inline bool seqBefore(uint16_t a, uint16_t b) { return (int16_t)(a - b) < 0; }

	/** Build one data notification from the head of the ring.
	 *  maxLen is the usable notification payload (ATT_MTU - 3). Returns the length (0 = nothing to send or no room).
	 *  Records that do not fit entirely are split into several records with the same t_ms.
	 *  The ring advances; on a failed send call ring.restore(cursor) with a cursor taken before. */
	inline size_t buildNotification(RecordRing &ring, uint16_t seq, uint8_t *out, size_t maxLen)
	{
		if (maxLen < NOTIFICATION_HEADER_LEN + RECORD_HEADER_LEN + 4) return 0;
		size_t len = NOTIFICATION_HEADER_LEN;
		out[0] = PROTOCOL_VERSION;
		put16(&out[1], seq);
		RecordRing::View r;
		while (ring.peek(r))
		{
			uint8_t sent = ring.partial();
			size_t remaining = (size_t)r.n - sent;
			size_t space = maxLen - len;
			if (space < RECORD_HEADER_LEN + 4) break;
			size_t fit = (space - RECORD_HEADER_LEN) / 4;
			size_t m = remaining < fit ? remaining : fit;
			out[len] = (uint8_t)r.tag[0]; out[len + 1] = (uint8_t)r.tag[1];
			out[len + 2] = (uint8_t)m;
			put32(&out[len + 3], r.t_ms);
			for (size_t i = 0; i < m; i++) memcpy(&out[len + RECORD_HEADER_LEN + 4 * i], &r.v[sent + i], 4);
			len += RECORD_HEADER_LEN + 4 * m;
			if (m == remaining) ring.popHead(); else ring.setPartial((uint8_t)(sent + m));
			if (m < remaining) break;	// frame is full
		}
		return len > NOTIFICATION_HEADER_LEN ? len : 0;
	}
}
