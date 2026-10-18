#pragma once
// Whole-band monitor for the automatic RF gain: the ADC peak over the full
// SDR bandwidth (strong signals anywhere overload the ADC, not only the TETRA
// channel) and a coarse spectrum around the carrier, giving the channel power
// against the local noise floor (SNR). Taps the IQ front end directly (no VFO).

#include <algorithm>
#include <atomic>
#include <cmath>
#include <complex>
#include <mutex>
#include <vector>
#include <signal_path/signal_path.h>
#include <gui/gui.h>
#include <dsp/sink/handler_sink.h>
#include "tx_analysis_math.h"

class TetraBandMonitor {
public:
    struct Result {
        bool valid = false;
        float peakDbfs = NAN;   // max |I|,|Q| over the last second, whole bandwidth
        float chanDb = NAN;     // power density in the TETRA channel (dB/bin)
        float floorDb = NAN;    // noise floor around the carrier (dB/bin, median)
        float snrDb = NAN;      // chanDb - floorDb
    };

    static constexpr int NFFT = 4096;
    static constexpr int NCOLLECT = 16 * NFFT;

    ~TetraBandMonitor() { stop(); }

    void loop(bool enable) {
        if (enable && !running) { start(); }
        else if (!enable && running) { stop(); }
    }

    bool isRunning() const { return running; }

    // Once per second from the GUI thread. carrierOffsetHz: carrier relative to
    // the SDR centre frequency.
    Result update(double carrierOffsetHz) {
        if (!running) { res = Result(); return res; }
        std::vector<std::complex<float>> x;
        float pk;
        {
            std::lock_guard<std::mutex> lck(mtx);
            pk = peak;
            peak = 0.0f;
            if ((int)buf.size() >= NCOLLECT) { x.swap(buf); }
            buf.clear();
            collect = true;
        }
        res.peakDbfs = pk > 0.0f ? 20.0f * log10f(pk) : NAN;
        double fs = sigpath::iqFrontEnd.getEffectiveSamplerate();
        // the block was collected right after the previous update: keep it with
        // the centre frequency of that time for snapCarrier()
        if (x.size() >= (size_t)NCOLLECT) { lastX = x; lastCenter = collectCenter; lastFs = fs; }
        collectCenter = gui::waterfall.getCenterFrequency();
        if (x.size() >= (size_t)NCOLLECT && fs > 0) {
            std::vector<double> p;
            double enbw = 1.0;
            TxMath::welch(x, NFFT, p, enbw);
            double bw = fs / NFFT;
            int cb = NFFT / 2 + (int)std::lround(carrierOffsetHz / bw);
            int chHalf = (std::max)(1, (int)(7000.0 / bw));
            double chSum = 0.0;
            int chN = 0;
            for (int k = cb - chHalf; k <= cb + chHalf; k++) {
                if (k >= 0 && k < NFFT) { chSum += p[k]; chN++; }
            }
            // local floor: 30 kHz ... 400 kHz from the carrier, away from DC
            std::vector<double> fl;
            int lo = (int)(30000.0 / bw), hi = (int)(400000.0 / bw), dc = (int)(20000.0 / bw) + 1;
            for (int k = 0; k < NFFT; k++) {
                int d = std::abs(k - cb);
                if (d < lo || d > hi || std::abs(k - NFFT / 2) <= dc) { continue; }
                fl.push_back(p[k]);
            }
            if (chN > 0 && fl.size() > 16) {
                std::nth_element(fl.begin(), fl.begin() + fl.size() / 2, fl.end());
                double f = (std::max)(fl[fl.size() / 2], 1e-30);
                double c = (std::max)(chSum / chN, 1e-30);
                res.chanDb = (float)(10.0 * std::log10(c));
                res.floorDb = (float)(10.0 * std::log10(f));
                res.snrDb = res.chanDb - res.floorDb;
            }
        }
        res.valid = !std::isnan(res.peakDbfs);
        return res;
    }

    Result last() const { return res; }

    // Snap to carrier: strongest TETRA-like signal (18 kHz occupied) within
    // +-25 kHz of 'clickOffsetHz' (relative to the SDR centre). Spectrum of the
    // last raw block evaluated directly around the click (Hann window, 500 Hz
    // steps), centre from the power centroid, put on the 6.25 kHz raster when
    // within 1 kHz of it. Returns 0 found, 1 no fresh block for this centre
    // frequency yet (retry), 2 no carrier.
    int snapCarrier(double clickOffsetHz, double& outOffsetHz) {
        double center = gui::waterfall.getCenterFrequency();
        if (lastX.empty() || !(lastFs > 0) || std::fabs(lastCenter - center) > 0.5) { return 1; }
        return TxMath::findCarrier(lastX, lastFs, center, clickOffsetHz, outOffsetHz) ? 0 : 2;
    }

private:
    void start() {
        stream = new dsp::stream<dsp::complex_t>();
        if (!sinkInit) { sink.init(stream, handler, this); sinkInit = true; }
        else { sink.setInput(stream); }
        {
            std::lock_guard<std::mutex> lck(mtx);
            buf.clear();
            peak = 0.0f;
            collect = true;
        }
        sink.start();
        sigpath::iqFrontEnd.bindIQStream(stream);
        running = true;
    }

    void stop() {
        if (!running) { return; }
        sigpath::iqFrontEnd.unbindIQStream(stream);
        sink.stop();
        delete stream;
        stream = nullptr;
        running = false;
        res = Result();
    }

    static void handler(dsp::complex_t* data, int count, void* ctx) {
        TetraBandMonitor* _this = (TetraBandMonitor*)ctx;
        float pk = 0.0f;
        for (int i = 0; i < count; i++) {
            pk = (std::max)(pk, (std::max)(std::fabs(data[i].re), std::fabs(data[i].im)));
        }
        std::lock_guard<std::mutex> lck(_this->mtx);
        _this->peak = (std::max)(_this->peak, pk);
        if (_this->collect) {
            size_t need = (size_t)NCOLLECT - _this->buf.size();
            size_t n = (std::min)(need, (size_t)count);
            for (size_t i = 0; i < n; i++) { _this->buf.emplace_back(data[i].re, data[i].im); }
            if (_this->buf.size() >= (size_t)NCOLLECT) { _this->collect = false; }
        }
    }

    bool running = false;
    dsp::stream<dsp::complex_t>* stream = nullptr;
    dsp::sink::Handler<dsp::complex_t> sink;
    bool sinkInit = false;
    std::mutex mtx;
    float peak = 0.0f;
    bool collect = true;
    std::vector<std::complex<float>> buf;
    std::vector<std::complex<float>> lastX;
    double lastCenter = 0.0, collectCenter = 0.0, lastFs = 0.0;
    Result res;
};
