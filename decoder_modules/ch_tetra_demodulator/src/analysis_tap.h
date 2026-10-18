#pragma once
// One shared tap at the TETRA VFO for the analyses (TX analysis, modulation
// accuracy, multipath) instead of one full-rate VFO each: at high SDR sample
// rates every VFO costs a frequency shift and a decimation chain at the full
// rate. The tap runs at 288 kS/s; subscribers get either these samples or
// 144 kS/s from an exact decimation by 2 (linear-phase FIR, Kaiser window,
// passband +-62.5 kHz flat to 2e-4 dB, stopband from 81.5 kHz below -100 dB),
// so the results do not depend on whether the tap is shared.

#include <algorithm>
#include <cmath>
#include <complex>
#include <mutex>
#include <string>
#include <vector>
#include <signal_path/signal_path.h>
#include <dsp/sink/handler_sink.h>
#include "analysis_tap_dsp.h"

class TetraAnalysisTap {
public:
    static constexpr double FS = 288000.0;
    static constexpr double FS_HALF = 144000.0;
    static constexpr double BW = 250000.0;
    using Fn = void (*)(const std::complex<float>* data, int count, void* ctx);

    ~TetraAnalysisTap() { stopTap(); }
    void setName(const std::string& n) { name = n; }

    // half = true: 144 kS/s, false: 288 kS/s. Starts the tap with the first subscriber.
    bool subscribe(void* ctx, bool half, Fn fn, double offset) {
        if (!running) { if (!startTap(offset)) { return false; } }
        else { setOffset(offset); }
        std::lock_guard<std::mutex> lck(subMtx);
        for (auto& s : subs) { if (s.ctx == ctx) { s.half = half; s.fn = fn; return true; } }
        subs.push_back({ ctx, half, fn });
        return true;
    }

    void unsubscribe(void* ctx) {
        bool empty;
        {
            std::lock_guard<std::mutex> lck(subMtx);
            subs.erase(std::remove_if(subs.begin(), subs.end(), [ctx](const Sub& s) { return s.ctx == ctx; }), subs.end());
            empty = subs.empty();
        }
        if (empty) { stopTap(); }
    }

    void setOffset(double offset) {
        if (!running || !vfo) { return; }
        if (std::fabs(offset - curOffset) < 0.5) { return; }
        curOffset = offset;
        vfo->setOffset(offset);
    }

    bool isRunning() const { return running; }
    // At SDR rates below 288 kS/s the tap interpolates; the users only need their
    // own band (TETRA channel +-20 kHz) inside the SDR span, as with separate taps.

private:
    struct Sub { void* ctx; bool half; Fn fn; };

    bool startTap(double offset) {
        vfo = sigpath::iqFrontEnd.addVFO(name, FS, BW, offset);
        if (!vfo) { return false; }
        curOffset = offset;
        decim.reset();
        if (!sinkInit) { sink.init(&vfo->out, handler, this); sinkInit = true; }
        else { sink.setInput(&vfo->out); }
        sink.start();
        running = true;
        return true;
    }

    void stopTap() {
        if (!running) { return; }
        sink.stop();
        sigpath::iqFrontEnd.removeVFO(name);
        vfo = nullptr;
        running = false;
    }

    static void handler(dsp::complex_t* data, int count, void* ctx) {
        TetraAnalysisTap* _this = (TetraAnalysisTap*)ctx;
        const std::complex<float>* x = (const std::complex<float>*)data;   // same layout (re, im)
        std::lock_guard<std::mutex> lck(_this->subMtx);
        bool needHalf = false;
        for (auto& s : _this->subs) { if (s.half) { needHalf = true; } else { s.fn(x, count, s.ctx); } }
        if (!needHalf) { return; }
        _this->halfBuf.clear();
        _this->decim.process(x, count, _this->halfBuf);
        if (_this->halfBuf.empty()) { return; }
        for (auto& s : _this->subs) { if (s.half) { s.fn(_this->halfBuf.data(), (int)_this->halfBuf.size(), s.ctx); } }
    }

    std::string name = "TETRA analysis";
    dsp::channel::RxVFO* vfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> sink;
    bool sinkInit = false;
    bool running = false;
    double curOffset = 0.0;
    std::mutex subMtx;
    std::vector<Sub> subs;
    tetra_tap::Decim2 decim;
    std::vector<std::complex<float>> halfBuf;
};
