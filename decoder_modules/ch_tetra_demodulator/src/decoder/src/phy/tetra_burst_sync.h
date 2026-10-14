#ifndef TETRA_BURST_SYNC_H
#define TETRA_BURST_SYNC_H

#include <stdint.h>

enum rx_state {
	RX_S_UNLOCKED,		/* we're completely unlocked */
	RX_S_KNOW_FSTART,	/* we know the next frame start */
	RX_S_LOCKED,		/* fully locked */
};

struct tetra_rx_state {
	enum rx_state state;
	unsigned int bits_in_buf;		/* how many bits are currently in bitbuf */
	uint8_t bitbuf[4096];
	unsigned int bitbuf_start_bitnum;	/* bit number at first element in bitbuf */
	unsigned int next_frame_start_bitnum;	/* frame start expected at this bitnum */

	void *burst_cb_priv;

	/* statistics (only ever increase, read by the GUI) */
	uint32_t stat_bursts;		/* bursts processed while locked */
	uint32_t stat_sync_lost;	/* transitions from locked to unlocked */
	uint32_t stat_known_bits;	/* known bits compared */
	uint32_t stat_known_err;	/* errors in known bits */
	uint32_t stat_tolerated;	/* bursts accepted with training sequence errors */

	/* sync loss diagnostics: after re-acquisition, compare the new frame
	 * timing with the old one. A non-zero remainder (mod one slot) means
	 * symbols were lost or inserted (e.g. dropped USB samples). */
	int resync_pending;
	unsigned int lost_next_bitnum;	/* where the next burst would have started */
	uint32_t stat_resync_slip;	/* re-locks with timing slip */
	uint32_t stat_resync_noslip;	/* re-locks on the old timing */
	int32_t last_slip_bits;		/* slip of the last re-lock, -255..254 bits */
	uint32_t last_resync_bits;	/* bits between loss and re-lock */
	uint32_t stat_resync_events;	/* incremented after every re-lock (for the GUI/log) */
};


/* input a raw bitstream into the tetra burst synchronizaer */
int tetra_burst_sync_in(struct tetra_rx_state *trs, uint8_t *bits, unsigned int len);

#endif /* TETRA_BURST_SYNC_H */
