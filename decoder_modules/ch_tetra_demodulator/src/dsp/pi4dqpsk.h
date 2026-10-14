#pragma once
#include <dsp/processor.h>

#include <fstream>
#include <iomanip>
#include <sstream>

#include <dsp/loop/phase_control_loop.h>
#include <dsp/taps/windowed_sinc.h>
#include <dsp/multirate/polyphase_bank.h>
#include <dsp/math/step.h>
#include <dsp/loop/costas.h>
#include <dsp/clock_recovery/mm.h>
#include <dsp/taps/root_raised_cosine.h>
#include <dsp/filter/fir.h>
#include <dsp/loop/fast_agc.h>
#include <dsp/loop/costas.h>
#include <dsp/clock_recovery/mm.h>
#include <math.h>
#include <atomic>

#include "fll.h"
#include "pi4dqpsk_costas.h"
#include "complex_fd.h"

namespace dsp {
    namespace demod {
        class PI4DQPSK : public Processor<complex_t, complex_t> {
            using base_type = Processor<complex_t, complex_t>;
        public:
            PI4DQPSK() {}

            PI4DQPSK(stream<complex_t>* in, double symbolrate, double samplerate, int rrcTapCount, double rrcBeta, double agcRate, double costasBandwidth, double fllBandwidth, double omegaGain, double muGain, double omegaRelLimit = 0.01) { init(in, symbolrate, samplerate, rrcTapCount, rrcBeta, agcRate, costasBandwidth, fllBandwidth, omegaGain, muGain); }

            ~PI4DQPSK();

            void init(stream<complex_t>* in, double symbolrate, double samplerate, int rrcTapCount, double rrcBeta, double agcRate, double costasBandwidth, double fllBandwidth, double omegaGain, double muGain, double omegaRelLimit = 0.01);

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

            void setSymbolrate(double symbolrate);
            void setSamplerate(double samplerate);
            void setRRCParams(int rrcTapCount, double rrcBeta);
            void setRRCTapCount(int rrcTapCount);
            void setRRCBeta(int rrcBeta);
            void setAGCRate(double agcRate);
            void setCostasBandwidth(double bandwidth);
            void setFllBandwidth(double fllBandwidth);
            void setMMParams(double omegaGain, double muGain, double omegaRelLimit = 0.01);
            void setOmegaGain(double omegaGain);
            void setMuGain(double muGain);
            void setOmegaRelLimit(double omegaRelLimit);

            void reset();

            int process(int count, const complex_t* in, complex_t* out);

            // --- Measurements (written by the DSP thread, read by the GUI) ---
            // Input power before AGC in dBFS (exponentially averaged)
            float getLevelDbfs() { return levelDbfs.load(); }
            // Highest sample magnitude in dBFS (held ~1 s) - ADC overload indicator
            float getPeakDbfs() { return peakDbfs.load(); }
            // Number of input samples processed so far (input rate check)
            uint64_t getSampleCount() { return sampleCount.load(); }
            // Carrier offset relative to the VFO center in Hz (FLL + Costas)
            float getCarrierOffsetHz() { return carrierOffsetHz.load(); }
            // Symbol rate deviation from nominal in ppm (clock recovery)
            float getSymbolRatePpm() { return symbolRatePpm.load(); }

            // Adaptive FLL: wide loop for acquisition, narrow loop once the
            // symbol sync is stable (lower phase jitter -> correct MER readings)
            void setTracking(bool narrow) { trackingRequested.store(narrow); }

            bool isTracking() { return trackingApplied.load(); }

        protected:
            std::atomic<bool> trackingRequested { false };
            std::atomic<bool> trackingApplied { false };
            double _fllWide = 0.02;
            double _fllNarrow = 0.006;
            std::atomic<float> levelDbfs { -150.0f };
            std::atomic<float> peakDbfs { -150.0f };
            std::atomic<uint64_t> sampleCount { 0 };
            double peakHold = 0.0;
            std::atomic<float> carrierOffsetHz { 0.0f };
            std::atomic<float> symbolRatePpm { 0.0f };
            double powerAvg = 0.0;
            double offsetAvg = 0.0;
            double omegaAvg = 0.0;
            bool measInit = false;

            double _symbolrate;
            double _samplerate;
            int _rrcTapCount;
            double _rrcBeta;

            loop::FLL fll;
            tap<float> rrcTaps;
            filter::FIR<complex_t, float> rrc;
            loop::FastAGC<complex_t> agc;
            loop::PI4DQPSK_COSTAS costas;
            clock_recovery::COMPLEX_FD recov;
        };
    }
}
