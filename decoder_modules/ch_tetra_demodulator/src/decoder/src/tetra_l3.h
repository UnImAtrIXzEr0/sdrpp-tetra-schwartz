#ifndef TETRA_L3_H
#define TETRA_L3_H
/* Downlink layer 3 decoding (LLC -> MLE -> CMCE / MM) for display purposes:
 * call setup / transmission grants (who talks in which group), releases,
 * SDS (status, text, LIP positions) and mobility management messages.
 * References: ETSI EN 300 392-2 clauses 14 (CMCE), 16 (MM), 18 (MLE),
 * 21 (LLC), 29 (SDS-TL), ETSI TS 100 392-18-1 (LIP). */

#include <stdint.h>

enum tetra_l3_type {
	TETRA_L3_CALL_SETUP = 1,	/* D-SETUP */
	TETRA_L3_CALL_CONNECT,		/* D-CONNECT */
	TETRA_L3_TX_GRANTED,		/* D-TX-GRANTED: someone got the floor */
	TETRA_L3_TX_CEASED,		/* D-TX-CEASED */
	TETRA_L3_CALL_RELEASE,		/* D-RELEASE / D-DISCONNECT */
	TETRA_L3_CALL_OTHER,		/* other CMCE call control PDUs */
	TETRA_L3_STATUS,		/* D-STATUS (pre-coded status) */
	TETRA_L3_SDS,			/* D-SDS-DATA (text, location, other) */
	TETRA_L3_MM,			/* mobility management (registration, group attach) */
};

struct tetra_l3_event {
	int type;			/* enum tetra_l3_type */
	uint8_t tn;			/* timeslot 1..4 */
	uint32_t addr_ssi;		/* MAC address (destination: ISSI or GSSI) */
	uint8_t addr_type;		/* MAC address type */
	int call_id;			/* -1 if not applicable */
	int comm_type;			/* -1 unknown, 0 individual, 1 group, 2 group acked, 3 broadcast */
	int encrypted_call;		/* end-to-end/air encryption flag from basic service info, -1 unknown */
	uint32_t party_ssi;		/* calling / transmitting party, 0 = not present */
	int status;			/* pre-coded status, -1 = none */
	int sds_pid;			/* SDS protocol identifier, -1 = none */
	int has_pos;
	double lat, lon;		/* degrees, if has_pos */
	int disconnect_cause;		/* -1 = none */
	int mm_type;			/* MM PDU type, -1 = none */
	char pdu[32];			/* PDU name */
	/* MM: group identities (attached talkgroups = scan list of the MS) */
	int ngroups;
	uint32_t groups[16];
	uint8_t group_detach[16];	/* 1 = detached, 0 = attached */
	int detach_all;			/* 1 = "detach all" / replace list */
	char text[320];			/* decoded text / description (UTF-8) */
};

struct tetra_mac_state;

/* bits: unpacked (one bit per byte) TM-SDU, len in bits */
void tetra_l3_rx_tm_sdu(struct tetra_mac_state *tms, const uint8_t *bits, int len,
			uint32_t addr_ssi, uint8_t addr_type, uint8_t tn);

/* public for tests */
void tetra_l3_parse_tl_sdu(struct tetra_mac_state *tms, const uint8_t *bits, int len,
			   uint32_t addr_ssi, uint8_t addr_type, uint8_t tn);

#endif
