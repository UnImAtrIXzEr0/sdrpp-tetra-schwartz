#pragma once
// TX analysis of a received TETRA carrier (spectrum based):
//  - averaged power spectrum (Welch, Blackman-Harris) of a 250 kHz wide tap
//  - channel power and adjacent channel power at +-25/50/75 kHz, measured
//    through the TETRA filter (root raised cosine, 18 ksym/s, alpha 0.35 ->
//    power weighting = raised cosine)
//  - occupied bandwidth (99 %), -26 dB bandwidth, shoulders and edge slope
//    compared with the ideal TETRA spectrum, carrier leakage line
//  - measurement floor: automatic (noise far from the carrier) and an
//    optional stored reference (e.g. terminated input)
// The tap is a DSP-only VFO of the IQ front end (not shown on the waterfall);
// the DSP thread only copies samples, all maths runs in a worker thread.

#include <dsp/types.h>
#include <dsp/stream.h>
#include "analysis_tap.h"
#include <signal_path/signal_path.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "tx_analysis_math.h"

struct TxAnalysisResult {
    bool valid = false;
    double fs = 250000.0;
    int fftSize = 0;
    double binHz = 0.0;
    double rbwHz = 0.0;
    int averages = 0;
    std::vector<float> psdDb;       // per-bin power, dBFS, fft-shifted (index 0 = -fs/2)
    std::vector<float> floorDb;     // stored reference floor (empty if none)
    double chanPowerDbfs = -200.0;  // TETRA-filtered channel power
    double peakDbfs = -200.0;       // sample peak of the analysis tap
    // ACP at -75,-50,-25,+25,+50,+75 kHz (dBc) and the floor for each
    double acpDbc[6] = { 0 };
    double acpFloorDbc[6] = { 0 };
    double obwHz = 0.0, obwIdealHz = 0.0;
    double bw26Hz = 0.0, bw26IdealHz = 0.0;
    double shoulderLoDb = 0.0, shoulderHiDb = 0.0;      // dB relative to in-band density
    double shoulderFloorDb = 0.0;                       // floor at the shoulders (same reference)
    double slopeLoDbPerKHz = 0.0, slopeHiDbPerKHz = 0.0; // 0 = not measurable
    double slopeIdealDbPerKHz = 0.0;
    double inbandDensityDb = -200.0;                    // dBFS per bin, |f| 1..5 kHz
    double carrierLeakDbc = 0.0;                        // spectral line at the carrier
    bool carrierLeakBelow = true;                       // true: only an upper bound
    double noiseDensityDb = -200.0;                     // dBFS per bin far from the carrier
};

class TetraTxAnalyzer {
public:
    static constexpr double FS = TetraAnalysisTap::FS;   // 288 kS/s from the shared tap

    ~TetraTxAnalyzer() { stop(); }

    bool isRunning() const { return running; }

    // name: unique DSP VFO name; offset: centre offset of the TETRA channel (Hz)
    void setTap(TetraAnalysisTap* t) { tap = t; }

    void start(double offset) {
        if (running || !tap) { return; }
        curOffset = offset;
        {
            std::lock_guard<std::mutex> lck(ringMtx);
            ring.assign(RING, std::complex<float>(0.0f, 0.0f));
            ringW = 0;
            ringFill = 0;
        }
        if (!tap->subscribe(this, false, handler, offset)) { return; }
        stopFlag = false;
        worker = std::thread(&TetraTxAnalyzer::work, this);
        running = true;
    }

    void stop() {
        if (!running) { return; }
        {
            std::lock_guard<std::mutex> lck(cvMtx);
            stopFlag = true;
        }
        cv.notify_all();
        if (worker.joinable()) { worker.join(); }
        tap->unsubscribe(this);
        running = false;
        std::lock_guard<std::mutex> lck(resMtx);
        res = TxAnalysisResult();
    }

    // follow the TETRA VFO (AFC, retuning)
    void setOffset(double offset) {
        if (!running) { return; }
        if (std::fabs(offset - curOffset) < 0.5) { return; }
        curOffset = offset;
        tap->setOffset(offset);
        resetAverage();
    }

    void setFftSize(int n) {
        if (n != 4096 && n != 8192 && n != 16384) { n = 8192; }
        if (n != fftSizeReq.load()) { fftSizeReq.store(n); resetAverage(); }
    }
    int getFftSize() const { return fftSizeReq.load(); }
    void setAverages(int n) { avgReq.store((std::max)(1, (std::min)(200, n))); }
    void setCarrierOffsetHz(double hz) { carrierHz.store(hz); }
    void resetAverage() { resetReq.store(true); }

    // reference floor: current averaged spectrum (e.g. with the input terminated)
    void storeFloor() { storeFloorReq.store(true); }
    void clearFloor() {
        std::lock_guard<std::mutex> lck(floorMtx);
        floorLin.clear();
    }
    bool hasFloor() {
        std::lock_guard<std::mutex> lck(floorMtx);
        return !floorLin.empty();
    }

    TxAnalysisResult get() {
        std::lock_guard<std::mutex> lck(resMtx);
        return res;
    }

private:
    using Metrics = TxMath::Metrics;
    static const size_t RING = 1 << 18;   // ~0.9 s at 288 kS/s
    static const size_t LEAK_N = 1 << 16; // 4.4 Hz bins for the carrier line

    static void handler(const std::complex<float>* data, int count, void* ctx) {
        TetraTxAnalyzer* _this = (TetraTxAnalyzer*)ctx;
        std::lock_guard<std::mutex> lck(_this->ringMtx);
        float pk = _this->peakAcc;
        for (int i = 0; i < count; i++) {
            _this->ring[_this->ringW] = data[i];
            _this->ringW = (_this->ringW + 1) & (RING - 1);
            float a = std::norm(data[i]);
            if (a > pk) { pk = a; }
        }
        _this->peakAcc = pk;
        _this->ringFill = (std::min)(RING, _this->ringFill + (size_t)count);
        _this->newSamples += count;
    }

    void work() {
        std::vector<double> avg, avgL;
        int avgN = 0;
        double obwI, bw26I, slopeI;
        TxMath::idealFigures(obwI, bw26I, slopeI);
        while (true) {
            {
                std::unique_lock<std::mutex> lck(cvMtx);
                if (cv.wait_for(lck, std::chrono::milliseconds(250), [this] { return stopFlag; })) { return; }
            }
            const int n = fftSizeReq.load();
            const size_t need = (size_t)(FS * 0.25) + n;   // newest 0.25 s
            std::vector<std::complex<float>> x;
            float peak;
            {
                std::lock_guard<std::mutex> lck(ringMtx);
                if (ringFill < need || newSamples < (size_t)(FS * 0.2)) { continue; }
                newSamples = 0;
                x.resize(need);
                size_t start = (ringW + RING - need) & (RING - 1);
                for (size_t i = 0; i < need; i++) { x[i] = ring[(start + i) & (RING - 1)]; }
                peak = peakAcc;
                peakAcc = 0.0f;
            }
            std::vector<double> p;
            double enbw = 2.0;
            TxMath::welch(x, n, p, enbw);
            // long FFT for the carrier line (newest LEAK_N samples, one segment)
            std::vector<double> pl;
            {
                std::vector<std::complex<float>> xl;
                {
                    std::lock_guard<std::mutex> lck(ringMtx);
                    xl.resize(LEAK_N);
                    size_t start = (ringW + RING - LEAK_N) & (RING - 1);
                    for (size_t i = 0; i < LEAK_N; i++) { xl[i] = ring[(start + i) & (RING - 1)]; }
                }
                double e2;
                TxMath::welch(xl, (int)LEAK_N, pl, e2);
            }
            if (resetReq.exchange(false) || (int)avg.size() != n) { avg.assign(n, 0.0); avgL.assign(LEAK_N, 0.0); avgN = 0; }
            int target = avgReq.load();
            avgN = (std::min)(avgN + 1, target);
            double a = 1.0 / avgN;
            for (int k = 0; k < n; k++) { avg[k] += a * (p[k] - avg[k]); }
            for (size_t k = 0; k < LEAK_N; k++) { avgL[k] += a * (pl[k] - avgL[k]); }

            if (storeFloorReq.exchange(false)) {
                std::lock_guard<std::mutex> lck(floorMtx);
                floorLin = avg;
            }
            std::vector<double> fl;
            {
                std::lock_guard<std::mutex> lck(floorMtx);
                if ((int)floorLin.size() == n) { fl = floorLin; }
            }
            Metrics m = TxMath::evaluate(avg, fl.empty() ? nullptr : &fl, FS, carrierHz.load());

            TxAnalysisResult r;
            r.valid = true;
            r.fs = FS;
            r.fftSize = n;
            r.binHz = FS / n;
            r.rbwHz = enbw * FS / n;
            r.averages = avgN;
            r.psdDb.resize(n);
            for (int k = 0; k < n; k++) { r.psdDb[k] = (float)(10.0 * std::log10((std::max)(avg[k], 1e-30))); }
            if (!fl.empty()) {
                r.floorDb.resize(n);
                for (int k = 0; k < n; k++) { r.floorDb[k] = (float)(10.0 * std::log10((std::max)(fl[k], 1e-30))); }
            }
            r.chanPowerDbfs = 10.0 * std::log10((std::max)(m.chan, 1e-30));
            r.peakDbfs = 10.0 * std::log10((std::max)((double)peak, 1e-30));
            for (int i = 0; i < 6; i++) { r.acpDbc[i] = m.acp[i]; r.acpFloorDbc[i] = m.acpFloor[i]; }
            r.obwHz = m.obw;
            r.obwIdealHz = obwI;
            r.bw26Hz = m.bw26;
            r.bw26IdealHz = bw26I;
            r.shoulderLoDb = m.shLo;
            r.shoulderHiDb = m.shHi;
            r.shoulderFloorDb = m.shFloor;
            r.slopeLoDbPerKHz = m.slLo;
            r.slopeHiDbPerKHz = m.slHi;
            r.slopeIdealDbPerKHz = slopeI;
            r.inbandDensityDb = 10.0 * std::log10((std::max)(m.inband, 1e-30));
            r.noiseDensityDb = 10.0 * std::log10((std::max)(m.noise, 1e-30));
            TxMath::carrierLeak(avgL, FS, carrierHz.load(), m.chan, avgN, r.carrierLeakDbc, r.carrierLeakBelow);
            std::lock_guard<std::mutex> lck(resMtx);
            res = std::move(r);
        }
    }

    TetraAnalysisTap* tap = nullptr;
    bool running = false;
    double curOffset = 0.0;

    std::mutex ringMtx;
    std::vector<std::complex<float>> ring;
    size_t ringW = 0, ringFill = 0, newSamples = 0;
    float peakAcc = 0.0f;

    std::thread worker;
    std::mutex cvMtx;
    std::condition_variable cv;
    bool stopFlag = false;

    std::atomic<int> fftSizeReq { 8192 };
    std::atomic<int> avgReq { 20 };
    std::atomic<double> carrierHz { 0.0 };
    std::atomic<bool> resetReq { false };
    std::atomic<bool> storeFloorReq { false };
    std::mutex floorMtx;
    std::vector<double> floorLin;

    std::mutex resMtx;
    TxAnalysisResult res;
};
