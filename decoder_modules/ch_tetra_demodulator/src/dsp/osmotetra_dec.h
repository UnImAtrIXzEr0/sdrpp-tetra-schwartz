#pragma once

#include <algorithm>
#include <dsp/processor.h>
#include <cmath>
#include <ctime>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <unistd.h>
#include <sys/time.h>
#endif

extern "C" {
    #include "tetra_common.h"
    #include <phy/tetra_burst.h>
    #include <phy/tetra_burst_sync.h>
    #include "c-code/channel.h"
    #include "c-code/source.h"
}

namespace dsp {

    class osmotetradec : public Processor<uint8_t, float> {
        using base_type = Processor<uint8_t, float>;
    public:
        osmotetradec() {}
        
        ~osmotetradec() {
            // Native Fix: Force base processing loops to stop running first
            base_type::stop();

            // Native Fix: 50ms platform-native sleep bypassing <chrono> and <thread>
            #if defined(_WIN32)
                ::Sleep(50); 
            #else
                ::usleep(50000); 
            #endif

            // NATIVE FIX: ONLY free the single heap array allocated with malloc
            if (conv_data != nullptr) {
                free(conv_data);
                conv_data = nullptr;
            }

            // DO NOT execute free() on tms, trs, tcs, t_display_st, or fragslots.
            // They are stack-allocated class members and free themselves natively.
        }

        osmotetradec(stream<uint8_t>* in) { init(in); }

        void init(stream<uint8_t>* in)  {
            // Bind our globally aligned stack structures directly to the operational pointers
            tms = &instance_tms;
            trs = &instance_trs;

            // Clear the memory pages cleanly
            memset(tms, 0, sizeof(struct tetra_mac_state));
            memset(trs, 0, sizeof(struct tetra_rx_state));

            // Set up the nested sub-structures natively on the stack
            tms->t_display_st = &instance_t_display_st;
            tms->fragslots = instance_fragslots;

            memset(tms->t_display_st, 0, sizeof(struct tetra_display_state));
            memset(tms->fragslots, 0, sizeof(struct fragslot) * FRAGSLOT_NR_SLOTS);

            // Initialize the Osmocom library states safely
            tetra_mac_state_init(tms);

            // Establish the secure private callback link
            trs->burst_cb_priv = tms;

            tms->put_voice_data = put_voice_data;
            tms->put_voice_data_ctx = this;
            tms->last_frame = 0;
            tms->curr_active_timeslot = 0;

            Init_Decod_Tetra();

            conv_data = (float*)malloc(sizeof(float) * STREAM_BUFFER_SIZE);
            memset(conv_data, 0, sizeof(float) * STREAM_BUFFER_SIZE);

            out_tmp_buff.init(32768);
            base_type::init(in);
        }

        //return current RX state. 0=unlocked, 1=know_next_start, 2=locked
        int getRxState() {
            switch(trs->state) {
                case RX_S_LOCKED:
                    return 2;
                case RX_S_KNOW_FSTART:
                    return 1;
                default:
                    return 0;
            }
        }

        // Extended cell information (SYNC, SYSINFO, D-NWRK-BROADCAST), read-only for display
        const struct tetra_display_state* getDisplay() { return tms->t_display_st; }
        void clearNeighbours() {
            tms->t_display_st->n_neigh = 0;
            tms->t_display_st->nwrk_count = 0;
            tms->t_display_st->reselect_valid = 0;
            tms->t_display_st->nettime_valid = 0;
        }
        int getCurrHyperframe() { return tms->t_display_st->curr_hyperframe; }
        int getCurrMultiframe() { return tms->t_display_st->curr_multiframe; }
        int getCurrFrame() { return tms->t_display_st->curr_frame; }
        int getTimeslotContent(int ts) { return tms->t_display_st->timeslot_content[ts]; }
        int getDlUsage() { return tms->t_display_st->dl_usage; }
        // timeslot occupancy counters (ts 0..3), see tetra_display_state
        uint32_t getDlUseCnt(int ts, int cat) { return tms->t_display_st->dl_use_cnt[ts & 3][cat & 3]; }
        uint32_t getUlUseCnt(int ts, int cat) { return tms->t_display_st->ul_use_cnt[ts & 3][(cat < 0 || cat > 2) ? 0 : cat]; }
        int getUlUsage() { return tms->t_display_st->ul_usage; }
        char getAccess1Code() { return tms->t_display_st->access1_code; }
        char getAccess2Code() { return tms->t_display_st->access2_code; }
        int getAccess1() { return tms->t_display_st->access1; }
        int getAccess2() { return tms->t_display_st->access2; }
        int getDlFreq() { return tms->t_display_st->dl_freq; }
        int getUlFreq() { return tms->t_display_st->ul_freq; }
        int getFreqBand() { return tms->t_display_st->freq_band; }
        int getDuplexIdx() { return tms->t_display_st->duplex_idx; }
        int getReverseOp() { return tms->t_display_st->reverse_op; }
        // Burst level statistics (monotonic counters, wrap-safe deltas)
        // Subscriber activity callback (called from the decoder thread)
        typedef void (*addr_cb_t)(void* ctx, uint8_t addr_type, uint32_t ssi, uint8_t usage_marker,
                                  uint16_t event_label, uint8_t tn, uint8_t encr_mode, uint8_t chan_alloc);
        void setAddressCallback(addr_cb_t cb, void* ctx) {
            tms->addr_cb_ctx = ctx;
            tms->addr_cb = cb;
        }

        typedef void (*l3_cb_t)(void* ctx, const struct tetra_l3_event* ev);
        void setL3Callback(l3_cb_t cb, void* ctx) {
            tms->l3_cb_ctx = ctx;
            tms->l3_cb = cb;
        }

        uint32_t getStatBursts() { return trs->stat_bursts; }
        uint32_t getStatSyncLost() { return trs->stat_sync_lost; }
        uint32_t getStatKnownBits() { return trs->stat_known_bits; }
        uint32_t getStatKnownErr() { return trs->stat_known_err; }
        uint32_t getStatTolerated() { return trs->stat_tolerated; }
        uint32_t getStatResyncEvents() { return trs->stat_resync_events; }
        uint32_t getStatResyncSlip() { return trs->stat_resync_slip; }
        uint32_t getStatResyncNoSlip() { return trs->stat_resync_noslip; }
        int getLastSlipBits() { return trs->last_slip_bits; }
        uint32_t getLastResyncBits() { return trs->last_resync_bits; }
        uint32_t getCrcOk(int ch) { return tms->t_display_st->crc_ok[ch & 3]; }
        uint32_t getCrcFail(int ch) { return tms->t_display_st->crc_fail[ch & 3]; }
        int getMcc() { return tms->t_display_st->mcc; }
        int getMnc() { return tms->t_display_st->mnc; }
        int getCc() { return tms->t_display_st->cc; }
        bool getLastCrcFail() { return tms->t_display_st->last_crc_fail; }
        bool getAdvancedLink() { return tms->t_display_st->advanced_link; }
        bool getAirEncryption() { return tms->t_display_st->air_encryption; }
        bool getSndcpData() { return tms->t_display_st->sndcp_data; }
        bool getCircuitData() { return tms->t_display_st->circuit_data; }
        bool getVoiceService() { return tms->t_display_st->voice_service; }
        bool getNormalMode() { return tms->t_display_st->normal_mode; }
        bool getMigrationSupported() { return tms->t_display_st->migration_supported; }
        bool getNeverMinimumMode() { return tms->t_display_st->never_minimum_mode; }
        bool getPriorityCell() { return tms->t_display_st->priority_cell; }
        bool getDeregMandatory() { return tms->t_display_st->dereg_mandatory; }
        bool getRegMandatory() { return tms->t_display_st->reg_mandatory; }
        inline int process(int count, const uint8_t* in, float* out)  {
            int outcnt = 0;

            if (trs == nullptr || tms == nullptr || count <= 0 || in == nullptr) {
                return 0;
            }

            // Unpack dibits into bits in chunks. Previously a fixed 8192-bit
            // static buffer silently dropped every symbol beyond 4095 per call,
            // which caused sporadic loss of burst sync on large input blocks.
            int i = 0;
            while (i < count) {
                int n = 0;
                while (i < count && n + 2 <= UNPACK_BUF_BITS) {
                    uint8_t symbol = in[i++];
                    unpacked_bits_buf[n++] = (symbol & 0b10) >> 1;
                    unpacked_bits_buf[n++] = symbol & 0b01;
                }
                const int SAFE_SLOT_BURST_SIZE = 510;
                for (int off = 0; off < n; off += SAFE_SLOT_BURST_SIZE) {
                    int chunk_size = (std::min)(SAFE_SLOT_BURST_SIZE, n - off);
                    trs->burst_cb_priv = tms;
                    tetra_burst_sync_in(trs, unpacked_bits_buf + off, chunk_size);
                }
            }

            if(out_tmp_buff.getReadable(false) > 0) {
                outcnt += out_tmp_buff.read(out, out_tmp_buff.getReadable(false));
            }
            outSymsCtr += outcnt;
            inSymsCtr += (count * 2); 
            
            int requiredOut = inSymsCtr * 8 / 36;
            int remainingOut = requiredOut - outSymsCtr;
            
            bool decoding = (tms->t_display_st->timeslot_content[0] == 4) | 
                            (tms->t_display_st->timeslot_content[1] == 4) | 
                            (tms->t_display_st->timeslot_content[2] == 4) | 
                            (tms->t_display_st->timeslot_content[3] == 4);
                            
            if(remainingOut > 0 && !decoding) {
                ::memset(&(out[outcnt]), 0, remainingOut * sizeof(float));
                outcnt += remainingOut;
            }
            
            outSymsCtr -= (std::min)(outSymsCtr, requiredOut);
            inSymsCtr -= requiredOut * 36 / 8;
            return outcnt;
        }

        int run()  {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            int outCount = process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            // Swap if some data was generated
            base_type::_in->flush();
            if (outCount) {
                if (!base_type::out.swap(outCount)) { return -1; }
            }
            return outCount;
        }

        static void put_voice_data(void* ctx, int count, int16_t* data) {
            osmotetradec* _this = (osmotetradec*) ctx;

            volk_16i_s32f_convert_32f(_this->conv_data, data, 32768.0f, count);
            if(_this->out_tmp_buff.getWritable(false) >= count) {
                _this->out_tmp_buff.write(_this->conv_data, count);
            }
        }
private:
        int inSymsCtr = 0;
        int outSymsCtr = 0;
        void *tetra_tall_ctx = NULL;
        struct tetra_rx_state *trs = NULL;
        struct tetra_mac_state *tms = NULL;
        float *conv_data = NULL;
        buffer::RingBuffer<float> out_tmp_buff;
        static const int UNPACK_BUF_BITS = 8192;
        uint8_t unpacked_bits_buf[UNPACK_BUF_BITS];
        struct tetra_mac_state instance_tms;
        struct tetra_rx_state instance_trs;
        struct tetra_display_state instance_t_display_st;
        struct fragslot instance_fragslots[FRAGSLOT_NR_SLOTS];
    };

}
