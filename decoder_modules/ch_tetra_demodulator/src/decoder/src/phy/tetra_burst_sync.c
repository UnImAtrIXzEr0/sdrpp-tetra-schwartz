/* Implementation of TETRA burst synchronization */

/* (C) 2011 by Harald Welte <laforge@gnumonks.org>
 * All Rights Reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

// #include <osmocom/core/utils.h>

//#define DEBUG

#include <tetra_common.h>
#include <phy/tetra_burst.h>
#include <tetra_tdma.h>
#include <phy/tetra_burst_sync.h>

struct tetra_phy_state t_phy_state;

void tetra_burst_rx_cb(const uint8_t *burst, unsigned int len, enum tetra_train_seq type, void *priv);

static void make_bitbuf_space(struct tetra_rx_state *trs, unsigned int len)
{
	unsigned int bitbuf_space = sizeof(trs->bitbuf) - trs->bits_in_buf;

	if (bitbuf_space < len) {
		unsigned int delta = len - bitbuf_space;

		DEBUGP("bitbuf left: %u, shrinking by %u\n", bitbuf_space, delta);
		memmove(trs->bitbuf, trs->bitbuf + delta, trs->bits_in_buf - delta);
		trs->bits_in_buf -= delta;
		trs->bitbuf_start_bitnum += delta;
		bitbuf_space = sizeof(trs->bitbuf) - trs->bits_in_buf;
	}
}

/* input a raw bitstream into the tetra burst synchronizaer */
int tetra_burst_sync_in(struct tetra_rx_state *trs, uint8_t *bits, unsigned int len)
{
	int rc;
	unsigned int train_seq_offs;

	DEBUGP("burst_sync_in: %u bits, state %u\n", len, trs->state);

	/* First: append the data to the bitbuf */
	make_bitbuf_space(trs, len);
	memcpy(trs->bitbuf + trs->bits_in_buf, bits, len);
	trs->bits_in_buf += len;

	switch (trs->state) {
	case RX_S_UNLOCKED:
		if (trs->bits_in_buf < TETRA_BITS_PER_TS*2) {
			/* wait for more bits to arrive */
			DEBUGP("-> waiting for more bits to arrive\n");
			return len;
		}
		DEBUGP("-> trying to find training sequence between bit %u and %u\n",
			trs->bitbuf_start_bitnum, trs->bits_in_buf);
		rc = tetra_find_train_seq(trs->bitbuf, trs->bits_in_buf,
					  (1 << TETRA_TRAIN_SYNC), &train_seq_offs);
		if (rc < 0)
			return rc;
		// printf("found SYNC training sequence in bit #%u\n", train_seq_offs);
		trs->state = RX_S_KNOW_FSTART;
		trs->next_frame_start_bitnum = trs->bitbuf_start_bitnum + train_seq_offs + 296;
		if (trs->resync_pending) {
			unsigned int d = trs->next_frame_start_bitnum - trs->lost_next_bitnum;
			int slip = (int)(d % TETRA_BITS_PER_TS);
			if (slip >= TETRA_BITS_PER_TS / 2)
				slip -= TETRA_BITS_PER_TS;
			trs->last_slip_bits = slip;
			trs->last_resync_bits = d;
			if (slip)
				trs->stat_resync_slip++;
			else
				trs->stat_resync_noslip++;
			trs->stat_resync_events++;
			trs->resync_pending = 0;
		}
#if 0
		if (train_seq_offs < 214) {
			/* not enough leading bits for start of burst */
			/* we just drop everything that we received so far */
			trs->bitbuf_start_bitnum += trs->bits_in_buf;
			trs->bits_in_buf = 0;
		}
#endif
		break;
	case RX_S_KNOW_FSTART:
		/* we are locked, i.e. already know when the next frame should start */
		if (trs->bitbuf_start_bitnum + trs->bits_in_buf < trs->next_frame_start_bitnum)
			return 0;
		else {
			/* shift start of frame to start of bitbuf */
			int offset = trs->next_frame_start_bitnum - trs->bitbuf_start_bitnum;
			int bits_remaining = trs->bits_in_buf - offset;

			memmove(trs->bitbuf, trs->bitbuf+offset, bits_remaining);
			trs->bits_in_buf = bits_remaining;
			trs->bitbuf_start_bitnum += offset;

			trs->next_frame_start_bitnum += TETRA_BITS_PER_TS;
			trs->state = RX_S_LOCKED;
		}
	case RX_S_LOCKED:
		if (trs->bits_in_buf < TETRA_BITS_PER_TS) {
			/* not sufficient data for the full frame yet */
			return len;
		} else {
			/* we have successfully received (at least) one frame */
			tetra_tdma_time_add_tn(&t_phy_state.time, 1);
			// printf("\nBURST");
			DEBUGP(": %s", osmo_ubit_dump(trs->bitbuf, TETRA_BITS_PER_TS));
			// printf("\n");
			{
				unsigned int kb = 0, eb = 0;
				rc = tetra_classify_locked_burst(trs->bitbuf, trs->bits_in_buf, &kb, &eb);
				if (rc >= 0) {
					trs->stat_bursts++;
					trs->stat_known_bits += kb;
					trs->stat_known_err += eb;
					if (eb)
						trs->stat_tolerated++;
					tetra_burst_rx_cb(trs->bitbuf, TETRA_BITS_PER_TS, rc, trs->burst_cb_priv);
				} else {
					trs->stat_sync_lost++;
					trs->state = RX_S_UNLOCKED;
					trs->resync_pending = 1;
					/* this burst should have started at bitbuf_start_bitnum */
					trs->lost_next_bitnum = trs->bitbuf_start_bitnum;
				}
			}

			/* move remainder to start of buffer */
			trs->bits_in_buf -= TETRA_BITS_PER_TS;
			memmove(trs->bitbuf, trs->bitbuf+TETRA_BITS_PER_TS, trs->bits_in_buf);
			trs->bitbuf_start_bitnum += TETRA_BITS_PER_TS;
			trs->next_frame_start_bitnum += TETRA_BITS_PER_TS;
		}
		break;

	}
	return len;
}
