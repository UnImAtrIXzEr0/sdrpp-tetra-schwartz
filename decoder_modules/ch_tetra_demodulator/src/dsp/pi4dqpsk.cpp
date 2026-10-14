#include "pi4dqpsk.h"
#include <algorithm>
#include <cmath>

namespace dsp {
    namespace demod {
        PI4DQPSK::~PI4DQPSK() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
            taps::free(rrcTaps);
        }

        void PI4DQPSK::init(stream<complex_t>* in, double symbolrate, double samplerate, int rrcTapCount, double rrcBeta, double agcRate, double costasBandwidth, double fllBandwidth, double omegaGain, double muGain, double omegaRelLimit) {
            _symbolrate = symbolrate;
            _samplerate = samplerate;
            _rrcTapCount = rrcTapCount;
            _rrcBeta = rrcBeta;

            _fllWide = fllBandwidth;
            _fllNarrow = (std::min)(fllBandwidth, 0.006);
            trackingApplied = false;
            fll.init(NULL, fllBandwidth, _symbolrate, _samplerate, _rrcTapCount, _rrcBeta, 0, -FL_M_PI/2.0f, FL_M_PI/2.0f);
            rrcTaps = taps::rootRaisedCosine<float>(_rrcTapCount, _rrcBeta, _symbolrate, _samplerate);
            rrc.init(NULL, rrcTaps);
            agc.init(NULL, 1.0, 10e6, agcRate);
            costas.init(NULL, costasBandwidth, 0, 0, -FL_M_PI/10.0f, FL_M_PI/10.0f); //frequency range limit here is REQUIRED!!!
            recov.init(NULL, _samplerate / _symbolrate,  omegaGain, muGain, omegaRelLimit);

            rrc.out.free();
            agc.out.free();
            costas.out.free();
            recov.out.free();

            base_type::init(in);
        }

        void PI4DQPSK::setSymbolrate(double symbolrate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _symbolrate = symbolrate;
            taps::free(rrcTaps);
            rrcTaps = taps::rootRaisedCosine<float>(_rrcTapCount, _rrcBeta, _symbolrate, _samplerate);
            rrc.setTaps(rrcTaps);
            recov.setOmega(_samplerate / _symbolrate);
            base_type::tempStart();
        }

        void PI4DQPSK::setSamplerate(double samplerate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _samplerate = samplerate;
            taps::free(rrcTaps);
            rrcTaps = taps::rootRaisedCosine<float>(_rrcTapCount, _rrcBeta, _symbolrate, _samplerate);
            rrc.setTaps(rrcTaps);
            recov.setOmega(_samplerate / _symbolrate);
            base_type::tempStart();
        }

        void PI4DQPSK::setRRCParams(int rrcTapCount, double rrcBeta) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            _rrcTapCount = rrcTapCount;
            _rrcBeta = rrcBeta;
            taps::free(rrcTaps);
            rrcTaps = taps::rootRaisedCosine<float>(_rrcTapCount, _rrcBeta, _symbolrate, _samplerate);
            rrc.setTaps(rrcTaps);
            base_type::tempStart();
        }

        void PI4DQPSK::setRRCTapCount(int rrcTapCount) {
            setRRCParams(rrcTapCount, _rrcBeta);
        }

        void PI4DQPSK::setRRCBeta(int rrcBeta) {
            setRRCParams(_rrcTapCount, rrcBeta);
        }

        void PI4DQPSK::setAGCRate(double agcRate) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            agc.setRate(agcRate);
        }

        void PI4DQPSK::setCostasBandwidth(double bandwidth) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            costas.setBandwidth(bandwidth);
        }

        void PI4DQPSK::setFllBandwidth(double fllBandwidth) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _fllWide = fllBandwidth;
            _fllNarrow = (std::min)(fllBandwidth, 0.006);
            fll.setBandwidth(trackingApplied ? _fllNarrow : _fllWide);
        }

        void PI4DQPSK::setMMParams(double omegaGain, double muGain, double omegaRelLimit) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            recov.setOmegaGain(omegaGain);
            recov.setMuGain(muGain);
            recov.setOmegaRelLimit(omegaRelLimit);
        }

        void PI4DQPSK::setOmegaGain(double omegaGain) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            recov.setOmegaGain(omegaGain);
        }

        void PI4DQPSK::setMuGain(double muGain) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            recov.setMuGain(muGain);
        }

        void PI4DQPSK::setOmegaRelLimit(double omegaRelLimit) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            recov.setOmegaRelLimit(omegaRelLimit);
        }

        void PI4DQPSK::reset() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            base_type::tempStop();
            fll.reset();
            rrc.reset();
            agc.reset();
            costas.reset();
            recov.reset();
            measInit = false;
            trackingRequested.store(false);
            trackingApplied = false;
            fll.setBandwidth(_fllWide);
            base_type::tempStart();
        }

        int PI4DQPSK::process(int count, const complex_t* in, complex_t* out) {
            sampleCount.fetch_add((uint64_t)count, std::memory_order_relaxed);
            // Apply FLL bandwidth changes in the DSP thread only
            bool wantNarrow = trackingRequested.load();
            if (wantNarrow != trackingApplied) {
                fll.setBandwidth(wantNarrow ? _fllNarrow : _fllWide);
                trackingApplied = wantNarrow;
            }

            // Input power before AGC (block mean, then exponential average)
            if (count > 0) {
                double p = 0.0, pk = 0.0;
                for (int i = 0; i < count; i++) {
                    double e = (double)in[i].re * in[i].re + (double)in[i].im * in[i].im;
                    p += e;
                    if (e > pk) { pk = e; }
                }
                // peak hold with ~1 s release
                double rel = (std::min)(1.0, (double)count / _samplerate);
                peakHold = (pk > peakHold) ? pk : peakHold * (1.0 - rel) + pk * rel;
                peakDbfs.store(peakHold > 1e-20 ? (float)(10.0 * log10(peakHold)) : -200.0f);
                p /= (double)count;
                // ~0.5 s time constant at 72 kS/s regardless of block size
                double a = (std::min)(1.0, (double)count / (0.5 * _samplerate));
                powerAvg = measInit ? powerAvg + a * (p - powerAvg) : p;
            }

            int ret = count;
            ret = agc.process(ret, (complex_t*) in, out);
            ret = fll.process(ret, out, out);
            ret = rrc.process(ret, out, out);
            ret = recov.process(ret, out, out);
            ret = costas.process(ret, out, out);

            // Carrier offset: FLL runs at the sample rate, Costas at the symbol rate
            double offs = (double)fll.getFreq() * _samplerate / (2.0 * 3.14159265358979323846)
                        + (double)costas.getFreq() * _symbolrate / (2.0 * 3.14159265358979323846);
            double om = recov.getOmegaCurrent();
            if (!measInit) {
                offsetAvg = offs;
                omegaAvg = om;
                measInit = true;
            } else {
                double a = (std::min)(1.0, (double)count / (2.0 * _samplerate));
                offsetAvg += a * (offs - offsetAvg);
                omegaAvg += a * (om - omegaAvg);
            }
            levelDbfs.store(powerAvg > 1e-20 ? (float)(10.0 * log10(powerAvg)) : -200.0f);
            carrierOffsetHz.store((float)offsetAvg);
            double nominal = _samplerate / _symbolrate;
            // omega = samples per symbol; larger omega -> slower symbol clock
            symbolRatePpm.store(omegaAvg > 0 ? (float)((nominal / omegaAvg - 1.0) * 1e6) : 0.0f);
            return ret;
        }
    }
}
