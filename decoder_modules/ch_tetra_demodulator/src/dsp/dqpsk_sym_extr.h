#pragma once
#include <dsp/processor.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#include <dsp/loop/phase_control_loop.h>
#include <dsp/taps/windowed_sinc.h>
#include <dsp/multirate/polyphase_bank.h>
#include <dsp/math/step.h>
#include <math.h>
#include <atomic>

#define SYNC_DETECT_BUF 4096
#define SYNC_DETECT_DISPLAY 256

namespace dsp {
    //Symbol mapper + differential decoder
    class DQPSKSymbolExtractor : public Processor<complex_t, uint8_t> {
        using base_type = Processor<complex_t, uint8_t>;
    public:
        int run() {
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

        int process(int count, const complex_t* in, uint8_t* out);

        bool sync = false;
        float standarderr = 0;

        // Modulation quality over the last SYNC_DETECT_BUF symbols
        // (decision-directed, amplitude-normalized, per ETSI EN 300 394-1 style)
        float getMerDb() { return merDb.load(); }
        float getEvmPct() { return evmPct.load(); }
        float getEvmPeakPct() { return evmPeakPct.load(); }
        // I/Q origin offset (carrier leakage of the transmitter or DC offset of
        // the receiver landing on the carrier) relative to the signal, in dB
        float getIqOffsetDb() { return iqOffsetDb.load(); }
        // MER with the I/Q origin offset removed (shows what the modulator does
        // apart from carrier leakage / receiver DC)
        float getMerExclOffsetDb() { return merExclDb.load(); }
        // Remove the measured I/Q offset before the symbol decisions (helps
        // decoding of transmitters with strong carrier leakage)
        void setIqOffsetCompensation(bool en) { iqComp.store(en); }
        bool getIqOffsetCompensation() { return iqComp.load(); }

    private:
        std::atomic<float> merDb { 0.0f };
        std::atomic<float> evmPct { 100.0f };
        std::atomic<float> evmPeakPct { 100.0f };
        std::atomic<float> iqOffsetDb { -100.0f };
        std::atomic<float> merExclDb { 0.0f };
        std::atomic<bool> iqComp { false };
        double sumRawRe = 0.0, sumRawIm = 0.0;
        double prevOffRe = 0.0, prevOffIm = 0.0; // offset estimate of the previous window (raw frame)
        double sumErrEx = 0.0;
        int rawRot = 0;
        double sumSig = 0.0;
        double sumErr = 0.0;
        double sumMag = 0.0;
        float peakErr = 0.0f;
        int merCount = 0;
        float lastScale = 1.0f;

        uint8_t prev = 0;
        float errorbuf[SYNC_DETECT_BUF];
        int errorptr = 0;
        int errordisplayptr = 0;
    };
}
