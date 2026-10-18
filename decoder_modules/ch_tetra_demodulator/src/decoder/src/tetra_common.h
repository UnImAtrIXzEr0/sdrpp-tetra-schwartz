#ifndef TETRA_COMMON_H
#define TETRA_COMMON_H

#include <stdint.h>
#include "tetra_mac_pdu.h"
#include <stdbool.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
// #include <osmocom/core/linuxlist.h>

#include "tetra_fragslot.h"


struct value_string {
	uint32_t value;		/*!< numeric value */
	const char *str;	/*!< human-readable string */
};

/*! get human-readable string or NULL for given value
 *  \param[in] vs Array of value_string tuples
 *  \param[in] val Value to be converted
 *  \returns pointer to human-readable string or NULL if val is not found
 */
const char *get_value_string_or_null(const struct value_string *vs,
				     uint32_t val);
const char *get_value_string(const struct value_string *vs, uint32_t val);


/*! Osmocom message buffer */
struct msgb {
	// struct llist_head list; /*!< linked list header */


	/* Part of which TRX logical channel we were received / transmitted */
	/* FIXME: move them into the control buffer */
	union {
		void *dst; /*!< reference of origin/destination */
		struct gsm_bts_trx *trx;
	};
	struct gsm_lchan *lchan; /*!< logical channel */

	unsigned char *l1h; /*!< pointer to Layer1 header (if any) */
	unsigned char *l2h; /*!< pointer to A-bis layer 2 header: OML, RSL(RLL), NS */
	unsigned char *l3h; /*!< pointer to Layer 3 header. For OML: FOM; RSL: 04.08; GPRS: BSSGP */
	unsigned char *l4h; /*!< pointer to layer 4 header */

	unsigned long cb[5]; /*!< control buffer */

	uint16_t data_len;   /*!< length of underlying data array */
	uint16_t len;	     /*!< length of bytes used in msgb */

	unsigned char *head;	/*!< start of underlying memory buffer */
	unsigned char *tail;	/*!< end of message in buffer */
	unsigned char *data;	/*!< start of message in buffer */
	unsigned char _data[0]; /*!< optional immediate data array */
};

/*! Allocate a new message buffer from given talloc context
 * \param[in] ctx talloc context from which to allocate
 * \param[in] size Length in octets, including headroom
 * \param[in] name Human-readable name to be associated with msgb
 * \returns dynamically-allocated \ref msgb
 *
 * This function allocates a 'struct msgb' as well as the underlying
 * memory buffer for the actual message data (size specified by \a size)
 * using the talloc memory context previously set by \ref msgb_set_talloc_ctx
 */
struct msgb *msgb_alloc_c(uint16_t size, const char *name);

/*! Allocate a new message buffer from tall_msgb_ctx
 * \param[in] size Length in octets, including headroom
 * \param[in] name Human-readable name to be associated with msgb
 * \returns dynamically-allocated \ref msgb
 *
 * This function allocates a 'struct msgb' as well as the underlying
 * memory buffer for the actual message data (size specified by \a size)
 * using the talloc memory context previously set by \ref msgb_set_talloc_ctx
 */
struct msgb *msgb_alloc(uint16_t size, const char *name);

/*! obtain L1 header of msgb */
#define msgb_l1(m)	((void *)((m)->l1h))
/*! obtain L2 header of msgb */
#define msgb_l2(m)	((void *)((m)->l2h))
/*! obtain L3 header of msgb */
#define msgb_l3(m)	((void *)((m)->l3h))
/*! obtain L4 header of msgb */
#define msgb_l4(m)	((void *)((m)->l4h))
/*! obtain SMS header of msgb */
#define msgb_sms(m)	msgb_l4(m)


/*! determine length of L1 message
 *  \param[in] msgb message buffer
 *  \returns size of L1 message in bytes
 *
 * This function computes the number of bytes between the tail of the
 * message and the layer 1 header.
 */
static inline unsigned int msgb_l1len(const struct msgb *msgb)
{
	assert(msgb->l1h);
	return (unsigned int) (msgb->tail - (uint8_t *)msgb_l1(msgb));
}

/*! determine length of L2 message
 *  \param[in] msgb message buffer
 *  \returns size of L2 message in bytes
 *
 * This function computes the number of bytes between the tail of the
 * message and the layer 2 header.
 */
static inline unsigned int msgb_l2len(const struct msgb *msgb)
{
	assert(msgb->l2h);
	return (unsigned int) (msgb->tail - (uint8_t *)msgb_l2(msgb));
}

/*! determine length of L3 message
 *  \param[in] msgb message buffer
 *  \returns size of L3 message in bytes
 *
 * This function computes the number of bytes between the tail of the
 * message and the layer 3 header.
 */
static inline unsigned int msgb_l3len(const struct msgb *msgb)
{
	assert(msgb->l3h);
	return (unsigned int) (msgb->tail - (uint8_t *)msgb_l3(msgb));
}

/*! determine length of L4 message
 *  \param[in] msgb message buffer
 *  \returns size of L4 message in bytes
 *
 * This function computes the number of bytes between the tail of the
 * message and the layer 4 header.
 */
static inline unsigned int msgb_l4len(const struct msgb *msgb)
{
	assert(msgb->l4h);
	return (unsigned int) (msgb->tail - (uint8_t *)msgb_l4(msgb));
}

/*! determine the length of the header
 *  \param[in] msgb message buffer
 *  \returns number of bytes between start of buffer and start of msg
 *
 * This function computes the length difference between the underlying
 * data buffer and the used section of the \a msgb.
 */
static inline unsigned int msgb_headlen(const struct msgb *msgb)
{
	return (unsigned int) (msgb->len - msgb->data_len);
}

static inline int msgb_tailroom(const struct msgb *msgb)
{
	return (unsigned int) ((msgb->head + msgb->data_len) - msgb->tail);
}

/*! append data to end of message buffer
 *  \param[in] msgb message buffer
 *  \param[in] len number of bytes to append to message
 *  \returns pointer to start of newly-appended data
 *
 * This function will move the \a tail pointer of the message buffer \a
 * len bytes further, thus enlarging the message by \a len bytes.
 *
 * The return value is a pointer to start of the newly added section at
 * the end of the message and can be used for actually filling/copying
 * data into it.
 */
static inline unsigned char *msgb_put(struct msgb *msgb, unsigned int len)
{
	unsigned char *tmp = msgb->tail;
	if (msgb_tailroom(msgb) < (int) len) {
		printf("Not enough tailroom msgb_put\n");
		abort();
	}
	msgb->tail += len;
	msgb->len += len;
	return tmp;
}









/*! primitive operation */
enum osmo_prim_operation {
	PRIM_OP_REQUEST,	/*!< request */
	PRIM_OP_RESPONSE,	/*!< response */
	PRIM_OP_INDICATION,	/*!< indication */
	PRIM_OP_CONFIRM,	/*!< confirm */
};
/*! Osmocom primitive header */
struct osmo_prim_hdr {
	unsigned int sap;	/*!< Service Access Point Identifier */
	unsigned int primitive;	/*!< Primitive number */
	enum osmo_prim_operation operation; /*! Primitive Operation */
	struct msgb *msg;	/*!< \ref msgb containing associated data.
       * Note this can be slightly confusing, as the \ref osmo_prim_hdr
       * is stored inside a \ref msgb, but then it contains a pointer
       * back to the msgb.  This is to simplify development: You can
       * pass around a \ref osmo_prim_hdr by itself, and any function
       * can autonomously resolve the underlying msgb, if needed (e.g.
       * for \ref msgb_free. */
};












#include "tetra_mac_pdu.h"

#ifdef DEBUG
#define DEBUGP(cformat, ...) printf(cformat, __VA_ARGS__)
#else
#define DEBUGP(cformat, ...) do { } while (0)
#endif

#define TETRA_SYM_PER_TS	255
#define TETRA_BITS_PER_TS	(TETRA_SYM_PER_TS*2)

/* Chapter 22.2.x */
enum tetra_log_chan {
	TETRA_LC_UNKNOWN,
	/* TMA SAP */
	TETRA_LC_SCH_F,
	TETRA_LC_SCH_HD,
	TETRA_LC_SCH_HU,
	TETRA_LC_STCH,
	TETRA_LC_SCH_P8_F,
	TETRA_LC_SCH_P8_HD,
	TETRA_LC_SCH_P8_HU,

	TETRA_LC_AACH,
	TETRA_LC_TCH,
	TETRA_LC_BSCH,
	TETRA_LC_BNCH,

	/* FIXME: QAM */
};

uint32_t bits_to_uint(const uint8_t *bits, unsigned int len);

#include "tetra_tdma.h"
struct tetra_phy_state {
	struct tetra_tdma_time time;
};
extern struct tetra_phy_state t_phy_state;

struct tetra_display_state {
	int curr_hyperframe;//
	int curr_multiframe;//
	int curr_frame;//
	int timeslot_content[4]; //0-other, 1-NORM1, 2-NORM2, 3-SYNC//
	int dl_usage;//
	int ul_usage;//
	char access1_code;//
	char access2_code;//
	int access1;//
	int access2;//
	int dl_freq;//
	int ul_freq;//
	int freq_band;		/* from SYSINFO, 0..15 */
	int duplex_idx;		/* duplex spacing table index from SYSINFO, 0..7 */
	int reverse_op;		/* reverse operation flag from SYSINFO */
	/* CRC statistics per channel: 0=BSCH(SB1) 1=SB2/BNCH 2=half-slot NDB 3=SCH/F (signalling only) */
	uint32_t crc_ok[4];
	uint32_t crc_fail[4];
	int mcc;//
	int mnc;//
	int cc;//
	bool last_crc_fail;//
	bool advanced_link;
	bool air_encryption;
	bool sndcp_data;
	bool circuit_data;
	bool voice_service;
	bool normal_mode;
	bool migration_supported;
	bool never_minimum_mode;
	bool priority_cell;
	bool dereg_mandatory;
	bool reg_mandatory;

	/* ---- extended cell information (display only, written by the decoder thread) ---- */
	/* SYNC (BSCH, CRC-valid only) */
	int sync_valid;
	int system_code;	/* 4 bit air interface version */
	int sharing_mode;	/* 0 continuous, 1 carrier, 2 MCCH, 3 traffic carrier sharing */
	int ts_reserved_frames;
	int uplane_dtx;
	int frame18_ext;
	int nb_cell_bcast;	/* D-MLE-SYNC neighbour cell broadcast (2 bits) */
	int cell_load_sync;	/* D-MLE-SYNC cell service level 0..3 */
	int late_entry;
	/* SYSINFO (BNCH) */
	int sysinfo_valid;
	int main_carrier;
	int freq_offset;	/* 0: 0, 1: +6.25, 2: -6.25, 3: +12.5 kHz */
	int num_csch;
	int ms_txpwr_max;	/* coded 1..7 */
	int rxlev_access_min;	/* coded 0..15 */
	int access_parameter;	/* coded 0..15 */
	int radio_dl_timeout;	/* coded 0..15 */
	int cck_valid;
	int cck_id;
	int la;
	int subscr_class;
	int option_field;
	uint32_t option_value;
	/* MLE D-NWRK-BROADCAST */
	uint32_t nwrk_count;
	int64_t nwrk_rx_time;	/* host time of the last one */
	int reselect_valid;
	int slow_resel_thr, slow_resel_hyst, fast_resel_thr, fast_resel_hyst;	/* coded 0..15 */
	int cell_load_nwrk;
	int nettime_valid;
	uint32_t nettime_utc;	/* 2 s units since start of year */
	int nettime_sign, nettime_offset, nettime_year;
	int64_t nettime_rx_time;
	/* timeslot occupancy from the ACCESS-ASSIGN PDU (AACH), frames 1..17,
	 * per timeslot 1..4 (index 0..3), counters since start:
	 * DL: 0 unallocated, 1 assigned control, 2 common control, 3 traffic
	 * UL: 0 unallocated, 1 control (random access / assigned), 2 traffic */
	uint32_t dl_use_cnt[4][4];
	uint32_t ul_use_cnt[4][3];
	int n_neigh;
	struct tetra_neigh_cell {
		int cell_id, resel_types, synced, load, main_carrier;
		int has_ext, band, offset, duplex, reverse;
		int mcc, mnc, la, txpwr, rxlev_min;	/* -1 = not sent */
		int64_t rx_time;
	} neigh[16];
};

struct tetra_l3_event;
struct tetra_mac_state {
	// struct llist_head voice_channels;
	struct {
		int is_traffic;
		bool blk1_stolen;
		bool blk2_stolen;
	} cur_burst;
	struct tetra_si_decoded last_sid;


	char *dumpdir;	/* Where to save traffic channel dump */
	int ssi;	/* SSI */
	int tsn;	/* Timeslon number */
	int usage_marker; /* Usage marker (if addressed)*/
	int addr_type;
	
	struct tetra_display_state *t_display_st;
	bool codec_first_pass;
	
	void (*put_voice_data)(void* ctx, int count, int16_t* data);
	/* called for every addressed MAC-RESOURCE PDU (subscriber activity) */
	void (*addr_cb)(void* ctx, uint8_t addr_type, uint32_t ssi, uint8_t usage_marker,
			uint16_t event_label, uint8_t tn, uint8_t encr_mode, uint8_t chan_alloc);
	void* addr_cb_ctx;
	/* decoded layer 3 messages (struct tetra_l3_event *) */
	void (*l3_cb)(void* ctx, const struct tetra_l3_event* ev);
	void* l3_cb_ctx;
	void* put_voice_data_ctx;
	int last_frame;
	int curr_active_timeslot;
	
	struct fragslot* fragslots;
};

extern struct tetra_display_state t_display_state;

void tetra_mac_state_init(struct tetra_mac_state *tms);

#define TETRA_CRC_OK	0x1d0f

uint32_t tetra_dl_carrier_hz(uint8_t band, uint16_t carrier, uint8_t offset);
uint32_t tetra_ul_carrier_hz(uint8_t band, uint16_t carrier, uint8_t offset,
			     uint8_t duplex, uint8_t reverse);

/* runtime-editable duplex spacing table (kHz, -1 = reserved) */
int32_t tetra_get_duplex_spacing_khz(uint8_t duplex, uint8_t band);
int32_t tetra_get_duplex_spacing_etsi_khz(uint8_t duplex, uint8_t band);
void tetra_set_duplex_spacing_khz(uint8_t duplex, uint8_t band, int32_t khz);
void tetra_reset_duplex_spacing_etsi(void);

const char *tetra_get_lchan_name(enum tetra_log_chan lchan);
const char *tetra_get_sap_name(uint8_t sap);
#endif
