/* Downlink layer 3 decoding for display: LLC -> MLE -> CMCE / MM, SDS, LIP.
 * Defensive bit reader: every read is bounds checked, a truncated or
 * corrupted PDU simply yields fewer fields, never an out-of-range access. */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#include "tetra_common.h"
#include "tetra_l3.h"

struct br {
	const uint8_t *b;
	int len;
	int pos;
	int err;
};

static uint32_t get(struct br *r, int n)
{
	uint32_t v = 0;
	if (n <= 0)
		return 0;
	if (r->pos + n > r->len) {
		r->err = 1;
		r->pos = r->len;
		return 0;
	}
	for (int i = 0; i < n; i++)
		v = (v << 1) | (r->b[r->pos + i] & 1);
	r->pos += n;
	return v;
}

static int left(const struct br *r) { return r->len - r->pos; }

static void emit(struct tetra_mac_state *tms, struct tetra_l3_event *ev)
{
	if (tms->l3_cb)
		tms->l3_cb(tms->l3_cb_ctx, ev);
}

static void ev_init(struct tetra_l3_event *ev, uint32_t ssi, uint8_t addr_type, uint8_t tn)
{
	memset(ev, 0, sizeof(*ev));
	ev->tn = tn;
	ev->addr_ssi = ssi;
	ev->addr_type = addr_type;
	ev->call_id = -1;
	ev->comm_type = -1;
	ev->encrypted_call = -1;
	ev->status = -1;
	ev->sds_pid = -1;
	ev->disconnect_cause = -1;
	ev->mm_type = -1;
}

/* ---------- text helpers (output UTF-8) ---------- */

static void put_utf8(char *out, int *o, int max, uint32_t cp)
{
	if (cp < 0x20 && cp != '\n') cp = ' ';
	if (cp == '\n') cp = ' ';
	if (cp < 0x80) {
		if (*o + 1 < max) out[(*o)++] = (char)cp;
	} else if (cp < 0x800) {
		if (*o + 2 < max) { out[(*o)++] = (char)(0xC0 | (cp >> 6)); out[(*o)++] = (char)(0x80 | (cp & 0x3F)); }
	} else {
		if (*o + 3 < max) {
			out[(*o)++] = (char)(0xE0 | (cp >> 12));
			out[(*o)++] = (char)(0x80 | ((cp >> 6) & 0x3F));
			out[(*o)++] = (char)(0x80 | (cp & 0x3F));
		}
	}
	out[*o] = 0;
}

/* GSM 03.38 default alphabet (positions differing from ASCII) */
static uint32_t gsm7(uint8_t c)
{
	static const uint16_t special[128] = {
		[0x00] = '@', [0x01] = 0xA3, [0x02] = '$', [0x03] = 0xA5, [0x04] = 0xE8, [0x05] = 0xE9, [0x06] = 0xF9, [0x07] = 0xEC,
		[0x08] = 0xF2, [0x09] = 0xC7, [0x0A] = '\n', [0x0B] = 0xD8, [0x0C] = 0xF8, [0x0D] = '\n', [0x0E] = 0xC5, [0x0F] = 0xE5,
		[0x10] = 0x394, [0x11] = '_', [0x12] = 0x3A6, [0x13] = 0x393, [0x14] = 0x39B, [0x15] = 0x3A9, [0x16] = 0x3A0, [0x17] = 0x3A8,
		[0x18] = 0x3A3, [0x19] = 0x398, [0x1A] = 0x39E, [0x1B] = ' ', [0x1C] = 0xC6, [0x1D] = 0xE6, [0x1E] = 0xDF, [0x1F] = 0xC9,
		[0x24] = 0xA4, [0x40] = 0xA1, [0x5B] = 0xC4, [0x5C] = 0xD6, [0x5D] = 0xD1, [0x5E] = 0xDC, [0x5F] = 0xA7,
		[0x60] = 0xBF, [0x7B] = 0xE4, [0x7C] = 0xF6, [0x7D] = 0xF1, [0x7E] = 0xFC, [0x7F] = 0xE0,
	};
	c &= 0x7F;
	if (special[c]) return special[c];
	return c;
}

static void decode_text(struct br *r, int tcs, char *out, int max)
{
	int o = 0;
	out[0] = 0;
	if (tcs == 0) {			/* 7-bit alphabet */
		while (left(r) >= 7)
			put_utf8(out, &o, max, gsm7((uint8_t)get(r, 7)));
	} else if (tcs == 26) {		/* UCS-2 */
		while (left(r) >= 16)
			put_utf8(out, &o, max, get(r, 16));
	} else {			/* 8-bit (ISO 8859-x; Latin-1 shown) */
		while (left(r) >= 8)
			put_utf8(out, &o, max, get(r, 8));
	}
	/* trim trailing blanks */
	while (o > 0 && out[o - 1] == ' ') out[--o] = 0;
}

/* ---------- LIP (TS 100 392-18-1) ---------- */

static int32_t sext(uint32_t v, int bits)
{
	if (v & (1u << (bits - 1)))
		return (int32_t)(v | ~((1u << bits) - 1));
	return (int32_t)v;
}

static int decode_lip(struct br *r, struct tetra_l3_event *ev)
{
	uint32_t pdu_type = get(r, 2);
	if (pdu_type == 0) {			/* short location report */
		get(r, 2);			/* time elapsed */
		int32_t lon = sext(get(r, 25), 25);
		int32_t lat = sext(get(r, 24), 24);
		uint32_t perr = get(r, 3);
		uint32_t hvel = get(r, 7);
		uint32_t dir = get(r, 4);
		if (r->err) return -1;
		ev->lon = lon * 360.0 / 33554432.0;
		ev->lat = lat * 180.0 / 16777216.0;
		ev->has_pos = 1;
		snprintf(ev->text, sizeof(ev->text), "LIP short: %.5f, %.5f (err class %u, speed code %u, dir %u)",
			 ev->lat, ev->lon, perr, hvel, dir);
		return 0;
	}
	if (pdu_type == 1) {
		uint32_t ext = get(r, 4);
		if (ext == 3) {			/* long location report */
			uint32_t time_type = get(r, 2);
			if (time_type == 1) get(r, 2);
			else if (time_type == 2) get(r, 22);
			uint32_t shape = get(r, 4);
			if (shape >= 1 && shape <= 6) {
				int32_t lon = sext(get(r, 25), 25);
				int32_t lat = sext(get(r, 24), 24);
				if (r->err) return -1;
				ev->lon = lon * 360.0 / 33554432.0;
				ev->lat = lat * 180.0 / 16777216.0;
				ev->has_pos = 1;
				snprintf(ev->text, sizeof(ev->text), "LIP long: %.5f, %.5f (shape %u)", ev->lat, ev->lon, shape);
				return 0;
			}
			snprintf(ev->text, sizeof(ev->text), "LIP long location report without position (shape %u)", shape);
			return 0;
		}
		snprintf(ev->text, sizeof(ev->text), "LIP PDU (extension %u)", ext);
		return 0;
	}
	snprintf(ev->text, sizeof(ev->text), "LIP PDU type %u", pdu_type);
	return 0;
}

/* ---------- SDS helpers ---------- */

/* protocol identifier names (EN 300 392-2, table 29.21) */
static const char *sds_pid_name(int pid)
{
	switch (pid) {
	case 0x01: return "OTAK";
	case 0x02: return "simple text";
	case 0x03: return "simple location";
	case 0x04: return "WAP";
	case 0x05: return "WCMP";
	case 0x06: return "M-DMO";
	case 0x07: return "PIN authentication";
	case 0x08: return "end-to-end encrypted";
	case 0x09: return "immediate text";
	case 0x0A: return "LIP";
	case 0x0B: return "net assist";
	case 0x0C: return "concatenated SDS";
	case 0x0D: return "DOTAM";
	case 0x82: return "text (SDS-TL)";
	case 0x83: return "location system (SDS-TL)";
	case 0x84: return "WAP (SDS-TL)";
	case 0x85: return "WCMP (SDS-TL)";
	case 0x86: return "M-DMO (SDS-TL)";
	case 0x88: return "end-to-end encrypted (SDS-TL)";
	case 0x89: return "immediate text (SDS-TL)";
	case 0x8A: return "message with user data header";
	case 0x8C: return "concatenated SDS (SDS-TL)";
	case 0x8D: return "DOTAM (SDS-TL)";
	default:
		if (pid >= 0xC0) return "user/vendor specific";
		if (pid >= 0x80) return "reserved (SDS-TL)";
		return "reserved";
	}
}

/* remaining bytes: printable text if it mostly is, otherwise hex */
static void bytes_text(struct br *r, char *out, int max)
{
	uint8_t b[96];
	int n = 0;
	while (n < (int)sizeof(b) && left(r) >= 8) b[n++] = (uint8_t)get(r, 8);
	int printable = 0;
	for (int i = 0; i < n; i++)
		if ((b[i] >= 0x20 && b[i] < 0x7F) || b[i] == '\r' || b[i] == '\n' || b[i] >= 0xA0) printable++;
	int o = 0;
	out[0] = 0;
	if (n > 0 && printable * 10 >= n * 8) {
		o = snprintf(out, max, "\"");
		for (int i = 0; i < n && o < max - 2; i++) put_utf8(out, &o, max - 1, b[i]);
		if (o < max - 1) { out[o++] = '"'; out[o] = 0; }
	} else {
		for (int i = 0; i < n && o < max - 4; i++)
			o += snprintf(out + o, max - o, "%s%02X", i ? " " : "", b[i]);
		if (left(r) >= 8 && o < max - 4) snprintf(out + o, max - o, " ...");
	}
}

/* user defined data of SDS types 1-3: hex, plus text if printable */
static void udata_text(uint32_t hi, uint32_t lo, int bits, char *out, int max)
{
	uint8_t b[8];
	int n = bits / 8;
	uint64_t v = ((uint64_t)hi << 32) | lo;
	for (int i = 0; i < n; i++) b[i] = (uint8_t)(v >> (8 * (n - 1 - i)));
	int o = snprintf(out, max, "0x");
	for (int i = 0; i < n; i++) o += snprintf(out + o, max - o, "%02X", b[i]);
	int pr = 1;
	for (int i = 0; i < n; i++) if (b[i] < 0x20 || b[i] >= 0x7F) pr = 0;
	if (pr && o < max - n - 4) {
		o += snprintf(out + o, max - o, " \"");
		for (int i = 0; i < n; i++) out[o++] = (char)b[i];
		out[o++] = '"';
		out[o] = 0;
	}
}

/* ---------- SDS type 4 user data ---------- */

static void decode_sds4(struct br *r, struct tetra_l3_event *ev)
{
	int pid = (int)get(r, 8);
	ev->sds_pid = pid;
	if (r->err) return;
	switch (pid) {
	case 0x02:	/* simple text messaging */
	case 0x09: {	/* simple immediate text messaging */
		get(r, 1);
		int tcs = (int)get(r, 7);
		char t[256];
		decode_text(r, tcs, t, sizeof(t));
		snprintf(ev->text, sizeof(ev->text), "Text: %s", t);
		break;
	}
	case 0x82:	/* text messaging (SDS-TL) */
	case 0x89: {	/* immediate text messaging (SDS-TL) */
		uint32_t mtype = get(r, 4);
		if (mtype != 0) {	/* SDS-REPORT / ACK / SHORT-REPORT */
			if (mtype == 1 || mtype == 2) {
				if (mtype == 1) { get(r, 1); get(r, 2); get(r, 1); }	/* ack req, reserved, storage */
				uint32_t st = get(r, 8), ref = get(r, 8);
				snprintf(ev->text, sizeof(ev->text), "SDS-TL %s for message %u: %s (0x%02X)", mtype == 1 ? "report" : "acknowledge", ref,
					 st == 0x00 ? "received" : st == 0x02 ? "consumed" : st < 0x20 ? "delivered" :
					 st < 0x40 ? "temporary error" : "failed", st);
			} else {
				snprintf(ev->text, sizeof(ev->text), "SDS-TL control (type %u)", mtype);
			}
			break;
		}
		get(r, 2);			/* delivery report request */
		get(r, 1);			/* service selection / short form report */
		uint32_t storage = get(r, 1);
		get(r, 8);			/* message reference */
		if (storage) {
			get(r, 5);		/* validity period */
			uint32_t fat = get(r, 3);	/* forward address type */
			if (fat == 0) get(r, 8);
			else if (fat == 1) get(r, 24);
			else if (fat == 3) get(r, 48);
			else { snprintf(ev->text, sizeof(ev->text), "SDS-TL text (external forward address)"); break; }
		}
		uint32_t ts_used = get(r, 1);
		int tcs = (int)get(r, 7);
		if (ts_used) get(r, 24);
		char t[256];
		decode_text(r, tcs, t, sizeof(t));
		snprintf(ev->text, sizeof(ev->text), "Text: %s", t);
		break;
	}
	case 0x0A:	/* location information protocol */
		decode_lip(r, ev);
		break;
	case 0x03: {	/* simple location system: coding + data (0 = NMEA 0183) */
		uint32_t coding = get(r, 8);
		char t[240];
		bytes_text(r, t, sizeof(t));
		snprintf(ev->text, sizeof(ev->text), "Location (%s): %s", coding == 0 ? "NMEA" : "other coding", t);
		break;
	}
	case 0x08:
	case 0x88:
		snprintf(ev->text, sizeof(ev->text), "SDS end-to-end encrypted (%d bytes, not readable)", left(r) / 8);
		break;
	default: {
		char t[240];
		bytes_text(r, t, sizeof(t));
		snprintf(ev->text, sizeof(ev->text), "SDS %s (PID 0x%02X): %s", sds_pid_name(pid), pid, t);
		break;
	}
	}
}

/* calling party: type identifier 2 bits, then SNA (8) / SSI (24) / TSI (24+24) */
static uint32_t read_party(struct br *r)
{
	uint32_t cpti = get(r, 2);
	uint32_t ssi = 0;
	if (cpti == 0) ssi = get(r, 8);
	else if (cpti == 1) ssi = get(r, 24);
	else if (cpti == 2) { ssi = get(r, 24); get(r, 24); }
	return r->err ? 0 : ssi;
}

static const char *cmce_name(int t)
{
	static const char *n[] = {
		"D-ALERT", "D-CALL-PROCEEDING", "D-CONNECT", "D-CONNECT-ACK", "D-DISCONNECT", "D-INFO",
		"D-RELEASE", "D-SETUP", "D-STATUS", "D-TX-CEASED", "D-TX-CONTINUE", "D-TX-GRANTED",
		"D-TX-WAIT", "D-TX-INTERRUPT", "D-CALL-RESTORE", "D-SDS-DATA", "D-FACILITY"
	};
	if (t >= 0 && t < (int)(sizeof(n) / sizeof(n[0]))) return n[t];
	return "D-CMCE-?";
}

static void set_service(struct tetra_l3_event *ev, uint32_t bsi)
{
	ev->encrypted_call = (bsi >> 4) & 1;
	ev->comm_type = (bsi >> 2) & 3;
}

static void parse_cmce(struct tetra_mac_state *tms, struct br *r, struct tetra_l3_event *ev)
{
	int t = (int)get(r, 5);
	if (r->err) return;
	snprintf(ev->pdu, sizeof(ev->pdu), "%s", cmce_name(t));
	switch (t) {
	case 7: {	/* D-SETUP */
		ev->type = TETRA_L3_CALL_SETUP;
		ev->call_id = (int)get(r, 14);
		get(r, 4); get(r, 1); get(r, 1);	/* timeout, hook, duplex */
		set_service(ev, get(r, 8));
		get(r, 2); get(r, 1); get(r, 4);	/* tx grant, tx req perm, priority */
		if (get(r, 1)) {			/* O-bit */
			if (get(r, 1)) get(r, 6);	/* notification indicator */
			if (get(r, 1)) get(r, 24);	/* temporary address */
			if (get(r, 1))			/* calling party (type id + address) */
				ev->party_ssi = read_party(r);
		}
		break;
	}
	case 2: {	/* D-CONNECT */
		ev->type = TETRA_L3_CALL_CONNECT;
		ev->call_id = (int)get(r, 14);
		get(r, 4); get(r, 1); get(r, 1); get(r, 2); get(r, 1); get(r, 1);
		if (get(r, 1)) {
			if (get(r, 1)) get(r, 4);		/* call priority */
			if (get(r, 1)) set_service(ev, get(r, 8));
		}
		break;
	}
	case 11: {	/* D-TX-GRANTED */
		ev->type = TETRA_L3_TX_GRANTED;
		ev->call_id = (int)get(r, 14);
		get(r, 2); get(r, 1); get(r, 1); get(r, 1);
		if (get(r, 1)) {
			if (get(r, 1)) get(r, 6);		/* notification indicator */
			if (get(r, 1)) ev->party_ssi = read_party(r);	/* transmitting party */
		}
		break;
	}
	case 9:		/* D-TX-CEASED */
		ev->type = TETRA_L3_TX_CEASED;
		ev->call_id = (int)get(r, 14);
		break;
	case 6:		/* D-RELEASE */
	case 4:		/* D-DISCONNECT */
		ev->type = TETRA_L3_CALL_RELEASE;
		ev->call_id = (int)get(r, 14);
		ev->disconnect_cause = (int)get(r, 5);
		break;
	case 8: {	/* D-STATUS */
		ev->type = TETRA_L3_STATUS;
		ev->party_ssi = read_party(r);
		ev->status = (int)get(r, 16);
		if (!r->err) {
			const char *sn = ev->status == 0 ? "EMERGENCY" :
					 ev->status < 0x8000 ? "reserved" :
					 ev->status < 0xFF00 ? "network/user defined" : "reserved";
			snprintf(ev->text, sizeof(ev->text), "Status %d (0x%04X, %s)", ev->status, ev->status, sn);
		}
		break;
	}
	case 15: {	/* D-SDS-DATA */
		ev->type = TETRA_L3_SDS;
		ev->party_ssi = read_party(r);
		uint32_t sdti = get(r, 2);
		char u[96];
		if (sdti == 0) { uint32_t v = get(r, 16); udata_text(0, v, 16, u, sizeof(u)); snprintf(ev->text, sizeof(ev->text), "SDS-1 (16 bit user data): %s", u); }
		else if (sdti == 1) { uint32_t v = get(r, 32); udata_text(0, v, 32, u, sizeof(u)); snprintf(ev->text, sizeof(ev->text), "SDS-2 (32 bit user data): %s", u); }
		else if (sdti == 2) { uint32_t a = get(r, 32), b = get(r, 32); udata_text(a, b, 64, u, sizeof(u)); snprintf(ev->text, sizeof(ev->text), "SDS-3 (64 bit user data): %s", u); }
		else {
			int l = (int)get(r, 11);
			if (r->err) break;
			struct br sub = { r->b + r->pos, (l < left(r)) ? l : left(r), 0, 0 };
			decode_sds4(&sub, ev);
		}
		break;
	}
	default:
		ev->type = TETRA_L3_CALL_OTHER;
		if (t <= 14 && t != 8) ev->call_id = (int)get(r, 14);
		break;
	}
	if (r->err) {
		/* truncated / corrupted: keep what was read, mark it */
		size_t l = strlen(ev->text);
		snprintf(ev->text + l, sizeof(ev->text) - l, "%s(truncated)", l ? " " : "");
	}
	emit(tms, ev);
}

static const char *mm_name(int t)
{
	static const char *n[] = {
		"D-OTAR", "D-AUTHENTICATION", "D-CK CHANGE DEMAND", "D-DISABLE", "D-ENABLE",
		"D-LOCATION UPDATE ACCEPT", "D-LOCATION UPDATE COMMAND", "D-LOCATION UPDATE REJECT", "D-MM-reserved",
		"D-LOCATION UPDATE PROCEEDING", "D-ATTACH/DETACH GROUP ID", "D-ATTACH/DETACH GROUP ID ACK",
		"D-MM STATUS", "D-MM-reserved", "D-MM-reserved", "D-MM PDU/FUNCTION NOT SUPPORTED"
	};
	return (t >= 0 && t < 16) ? n[t] : "D-MM-?";
}

/* Group identity downlink (type 4 element payload, EN 300 392-2 16.10.22):
 * number of repeated elements (6), then per element:
 *   attach/detach type identifier (1): 0 -> lifetime (2) + class of usage (3)
 *                                      1 -> detachment downlink (2)
 *   group identity address type (2): 0 GSSI(24) 1 GTSI(48) 2 vGSSI(24) 3 GTSI+vGSSI(72)
 * Returns 0 if the element parsed exactly into its length. */
static int parse_gid_downlink(struct br *r, int len, struct tetra_l3_event *ev)
{
	int end = r->pos + len;
	if (end > r->len) return -1;
	int n = (int)get(r, 6);
	for (int i = 0; i < n && !r->err && r->pos < end; i++) {
		uint32_t det = get(r, 1);
		if (det) get(r, 2); else get(r, 5);
		uint32_t at = get(r, 2);
		uint32_t gssi = get(r, 24);
		if (at == 1) get(r, 24);
		else if (at == 3) get(r, 48);
		if (r->err || r->pos > end) return -1;
		if (ev->ngroups < 16 && gssi && gssi <= 0xFFFFFF) {
			ev->groups[ev->ngroups] = gssi;
			ev->group_detach[ev->ngroups] = (uint8_t)det;
			ev->ngroups++;
		}
	}
	r->pos = end;
	return r->err ? -1 : 0;
}

/* Walk type 3/4 elements (M-bit, id 4, length 11, data) and pick up group lists */
static void parse_mm_elements(struct br *r, struct tetra_l3_event *ev, int depth)
{
	while (left(r) >= 16 && !r->err) {
		if (!get(r, 1)) break;				/* M-bit: no more elements */
		uint32_t id = get(r, 4);
		int len = (int)get(r, 11);
		if (r->err || len > left(r)) break;
		int start = r->pos;
		if (id == 7) {					/* group identity downlink */
			struct br sub = { r->b + start, len, 0, 0 };
			parse_gid_downlink(&sub, len, ev);
		} else if (id == 5 && depth == 0) {		/* group identity location accept */
			struct br sub = { r->b + start, len, 0, 0 };
			get(&sub, 1); get(&sub, 1);		/* accept/reject, reserved */
			if (get(&sub, 1))			/* O-bit */
				parse_mm_elements(&sub, ev, depth + 1);
		}
		r->pos = start + len;
	}
}

static void parse_mm_groups(struct br *r, int t, struct tetra_l3_event *ev)
{
	if (t == 5) {			/* D-LOCATION UPDATE ACCEPT */
		get(r, 3);		/* location update accept type */
		if (!get(r, 1)) return;	/* O-bit */
		if (get(r, 1)) get(r, 24);	/* SSI */
		if (get(r, 1)) get(r, 24);	/* address extension */
		if (get(r, 1)) get(r, 16);	/* subscriber class */
		if (get(r, 1)) get(r, 14);	/* energy saving information */
		if (get(r, 1)) get(r, 6);	/* SCCH information */
		parse_mm_elements(r, ev, 0);
	} else if (t == 10) {		/* D-ATTACH/DETACH GROUP IDENTITY */
		get(r, 1); get(r, 1);	/* report, ack request */
		ev->detach_all = (int)get(r, 1);	/* attach/detach mode: 1 = replace all */
		if (!get(r, 1)) return;
		parse_mm_elements(r, ev, 0);
	} else if (t == 11) {		/* D-ATTACH/DETACH GROUP IDENTITY ACK */
		get(r, 1); get(r, 1);
		if (!get(r, 1)) return;
		parse_mm_elements(r, ev, 0);
	}
}


/* ---------- MLE: D-NWRK-BROADCAST (EN 300 392-2 18.4.1.4.1) ---------- */

static int pbit(struct br *r) { return (int)get(r, 1) && !r->err; }

static void parse_neigh(struct br *r, struct tetra_neigh_cell *n)
{
	memset(n, 0, sizeof(*n));
	n->mcc = n->mnc = n->la = n->txpwr = n->rxlev_min = -1;
	n->cell_id = (int)get(r, 5);
	n->resel_types = (int)get(r, 2);
	n->synced = (int)get(r, 1);
	n->load = (int)get(r, 2);
	n->main_carrier = (int)get(r, 12);
	if (!pbit(r)) return;		/* O-bit */
	if (pbit(r)) {			/* main carrier number extension */
		n->has_ext = 1;
		n->band = (int)get(r, 4);
		n->offset = (int)get(r, 2);
		n->duplex = (int)get(r, 3);
		n->reverse = (int)get(r, 1);
	}
	if (pbit(r)) n->mcc = (int)get(r, 10);
	if (pbit(r)) n->mnc = (int)get(r, 14);
	if (pbit(r)) n->la = (int)get(r, 14);
	if (pbit(r)) n->txpwr = (int)get(r, 3);
	if (pbit(r)) n->rxlev_min = (int)get(r, 4);
	if (pbit(r)) get(r, 16);	/* subscriber class */
	if (pbit(r)) get(r, 12);	/* BS service details */
	if (pbit(r)) get(r, 5);		/* timeshare cell / security parameters */
	if (pbit(r)) get(r, 6);		/* TDMA frame offset */
}

static void parse_mle(struct tetra_mac_state *tms, struct br *r)
{
	struct tetra_display_state *d = tms->t_display_st;
	int t = (int)get(r, 3);
	if (r->err || !d || t != 2)	/* only D-NWRK-BROADCAST */
		return;
	uint32_t resel = get(r, 16);
	int load = (int)get(r, 2);
	if (r->err) return;
	int64_t now = (int64_t)time(NULL);
	d->slow_resel_thr = (resel >> 12) & 15;
	d->slow_resel_hyst = (resel >> 8) & 15;
	d->fast_resel_thr = (resel >> 4) & 15;
	d->fast_resel_hyst = resel & 15;
	d->reselect_valid = 1;
	d->cell_load_nwrk = load;
	d->nwrk_count++;
	d->nwrk_rx_time = now;
	if (!pbit(r)) return;		/* O-bit: no optional elements */
	if (pbit(r)) {			/* TETRA network time */
		uint32_t utc = get(r, 24);
		int sign = (int)get(r, 1);
		int off = (int)get(r, 6);
		int year = (int)get(r, 6);
		get(r, 11);
		if (!r->err && utc < 15811200u) {	/* < 366 days in 2 s units */
			d->nettime_utc = utc;
			d->nettime_sign = sign;
			d->nettime_offset = off;
			d->nettime_year = year;
			d->nettime_rx_time = now;
			d->nettime_valid = 1;
		}
	}
	if (pbit(r)) {			/* number of neighbour cells */
		int n = (int)get(r, 3);
		for (int i = 0; i < n && !r->err; i++) {
			struct tetra_neigh_cell nc;
			parse_neigh(r, &nc);
			if (r->err) break;
			nc.rx_time = now;
			/* merge by cell identifier + carrier, keep the list stable */
			int k;
			for (k = 0; k < d->n_neigh; k++)
				if (d->neigh[k].cell_id == nc.cell_id && d->neigh[k].main_carrier == nc.main_carrier)
					break;
			if (k == d->n_neigh) {
				if (d->n_neigh >= (int)(sizeof(d->neigh) / sizeof(d->neigh[0])))
					continue;
				d->n_neigh++;
			}
			d->neigh[k] = nc;
		}
	}
}

void tetra_l3_parse_tl_sdu(struct tetra_mac_state *tms, const uint8_t *bits, int len,
			   uint32_t addr_ssi, uint8_t addr_type, uint8_t tn)
{
	struct br r = { bits, len, 0, 0 };
	struct tetra_l3_event ev;
	ev_init(&ev, addr_ssi, addr_type, tn);

	uint32_t pd = get(&r, 3);	/* MLE protocol discriminator */
	if (r.err) return;
	switch (pd) {
	case 1: {			/* MM */
		int t = (int)get(&r, 4);
		if (r.err) return;
		ev.type = TETRA_L3_MM;
		ev.mm_type = t;
		snprintf(ev.pdu, sizeof(ev.pdu), "%s", mm_name(t));
		if (t == 5 || t == 10 || t == 11)
			parse_mm_groups(&r, t, &ev);
		{
			int o = 0;
			if (t == 5) o = snprintf(ev.text, sizeof(ev.text), "registration accepted");
			else if (t == 7) o = snprintf(ev.text, sizeof(ev.text), "registration rejected");
			else if (t == 10) o = snprintf(ev.text, sizeof(ev.text), "group attach/detach%s", ev.detach_all ? " (new list)" : "");
			else if (t == 11) o = snprintf(ev.text, sizeof(ev.text), "group attach/detach ack");
			for (int i = 0; i < ev.ngroups && o < (int)sizeof(ev.text) - 16; i++)
				o += snprintf(ev.text + o, sizeof(ev.text) - o, "%s%s%u", i ? ", " : (o ? ": " : ""),
					      ev.group_detach[i] ? "-" : "+", ev.groups[i]);
		}
		emit(tms, &ev);
		break;
	}
	case 2:				/* CMCE */
		parse_cmce(tms, &r, &ev);
		break;
	case 5:				/* MLE: network broadcast (neighbour cells, time) */
		parse_mle(tms, &r);
		break;
	default:
		break;			/* SNDCP, TETRA management: not shown */
	}
}

void tetra_l3_rx_tm_sdu(struct tetra_mac_state *tms, const uint8_t *bits, int len,
			uint32_t addr_ssi, uint8_t addr_type, uint8_t tn)
{
	if (!tms || !tms->l3_cb || !bits || len < 4)
		return;
	struct br r = { bits, len, 0, 0 };
	uint32_t t = get(&r, 4);	/* LLC PDU type */
	int fcs = 0;
	switch (t) {
	case 0: get(&r, 1); get(&r, 1); break;		/* BL-ADATA */
	case 1: get(&r, 1); break;			/* BL-DATA */
	case 2: break;					/* BL-UDATA */
	case 3: get(&r, 1); break;			/* BL-ACK */
	case 4: get(&r, 1); get(&r, 1); fcs = 1; break;	/* BL-ADATA + FCS */
	case 5: get(&r, 1); fcs = 1; break;
	case 6: fcs = 1; break;
	case 7: get(&r, 1); fcs = 1; break;
	default: return;				/* advanced link / L2 signalling */
	}
	int sdu_len = left(&r) - (fcs ? 32 : 0);
	if (r.err || sdu_len < 3)
		return;
	tetra_l3_parse_tl_sdu(tms, bits + r.pos, sdu_len, addr_ssi, addr_type, tn);
}
