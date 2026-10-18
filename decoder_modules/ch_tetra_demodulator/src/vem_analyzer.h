#pragma once
// Modulation accuracy of the downlink per EN 300 394-1 10.1.3 (see etsi_vem.h):
// own tap at the TETRA VFO (144 kS/s, DSP only), TETRA filter, slot grid from
// the training sequences, exact ideal symbols from CRC-checked bursts,
// statistics over the last 200 bursts (10.1.3 b).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <imgui.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/main_window.h>
#include <signal_path/signal_path.h>
#include <dsp/sink/handler_sink.h>
#include "tetra_colors.h"
#include "tetra_layout.h"
#include "etsi_vem.h"

using TetraVemResult = etsi_vem::Stats;

class TetraVemAnalyzer {
public:
    static constexpr double FS = etsi_vem::FS;

    ~TetraVemAnalyzer() { stop(); }
    void setName(const std::string& n) { name = n; }
    bool isRunning() const { return running; }
    std::string status;

    // freq: TETRA VFO; mcc/mnc/cc of the cell from the decoder (0: learn from BSCH)
    void loop(bool enable, double freq, uint16_t mcc, uint16_t mnc, uint8_t cc) {
        bool sdr = gui::mainWindow.sdrIsRunning();
        double center = gui::waterfall.getCenterFrequency();
        double sr = sigpath::iqFrontEnd.getSampleRate();
        double off = freq - center;
        status.clear();
        if (!enable) { stop(); return; }
        if (!sdr) { status = "SDR stopped"; stop(); return; }
        if (!(sr > 0) || std::fabs(off) > sr / 2.0 - 20000.0) { status = "TETRA VFO outside the SDR span"; stop(); return; }
        scrReq.store((mcc || mnc) ? ::tetra_scramb_get_init(mcc, mnc, cc) : 0u);
        if (!running) { start(off); }
        else if (std::fabs(off - curOffset) > 0.5) {
            curOffset = off;
            vfo->setOffset(off);
            if (std::fabs(off - resetOffset) > 2000.0) { resetOffset = off; resetReq.store(true); }   // retuned, not AFC
        }
    }

    void stop() {
        if (!running) { return; }
        {
            std::lock_guard<std::mutex> lck(cvMtx);
            stopFlag = true;
        }
        cv.notify_all();
        if (worker.joinable()) { worker.join(); }
        sink.stop();
        sigpath::iqFrontEnd.removeVFO(name);
        vfo = nullptr;
        running = false;
        std::lock_guard<std::mutex> lck(resMtx);
        res = TetraVemResult();
    }

    void reset() { resetReq.store(true); }

    TetraVemResult get() {
        std::lock_guard<std::mutex> lck(resMtx);
        return res;
    }

    // ---- GUI; 'etsiOk' / 'why' from the module (gain held, reference) ----
    bool draw(const std::string& id, float width, bool& run, bool gainHeld, int freqRef) {
        bool changed = false;
        ImVec4 val = tc::val(), good = tc::good(), bad = tc::bad(), warn = tc::warn();
        if (ImGui::Checkbox(("Modulation accuracy (EN 300 394-1, 10.1.3)##_vem_run_" + id).c_str(), &run)) { changed = true; }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Vector error per burst as in the conformance test: TETRA filter (root raised\n"
                              "cosine 0.35, 33 symbols), one frequency / phase / timing / amplitude /\n"
                              "origin offset per burst, no tracking loops. Ideal symbols are exact: only\n"
                              "bursts whose blocks pass the CRC are used and coded again bit by bit\n"
                              "(speech bursts are left out). Statistics over the last 200 bursts.\n"
                              "Limits (7.3.1.2): RMS < 10 %% in any burst, peak < 30 %%, residual carrier < 5 %%.");
        }
        if (!run) { return changed; }
        if (!status.empty()) { tw::TextColored(warn, "%s", status.c_str()); return changed; }
        TetraVemResult r = get();
        if (!r.gridValid) { tw::TextDisabled("looking for the slot grid..."); return changed; }
        if (!r.cellKnown) { tw::TextDisabled("waiting for a synchronisation burst (cell identity)..."); }
        tw::Text("Bursts:"); tw::SameTextColored(r.n >= etsi_vem::Stats::TARGET ? good : val, "%d/%d", r.n, etsi_vem::Stats::TARGET);
        tw::SameTextDisabled("(left out: %llu speech/CRC, %llu other)", (unsigned long long)(r.rejTraffic + r.rejCrc), (unsigned long long)r.rejOther);
        if (r.n == 0) { return changed; }
        auto lim = [&](double v, double l) { return v < l ? good : bad; };
        tw::Text("RMS vector error:"); tw::SameTextColored(val, "%.2f %%", r.rmsMean * 100.0);
        tw::SameText(" max"); tw::SameTextColored(lim(r.rmsMax, 0.10), "%.2f %%", r.rmsMax * 100.0); tw::SameTextDisabled("(< 10 %%)");
        tw::Text("Peak vector error:"); tw::SameTextColored(lim(r.peakMax, 0.30), "%.1f %%", r.peakMax * 100.0); tw::SameTextDisabled("(< 30 %%)");
        tw::Text("Residual carrier:"); tw::SameTextColored(val, "%.2f %%", r.c0Mean * 100.0);
        tw::SameText(" max"); tw::SameTextColored(lim(r.c0Max, 0.05), "%.2f %%", r.c0Max * 100.0); tw::SameTextDisabled("(< 5 %%)");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Origin offset of the constellation relative to the signal amplitude\n"
                              "(carrier leakage of the transmitter), %.1f dB.", 20.0 * std::log10((std::max)(r.c0Mean, 1e-6)));
        }
        tw::Text("Frequency (fit):"); tw::SameTextColored(val, "%+.1f Hz", r.freqMean); tw::SameTextDisabled("+-%.1f Hz vs. the TETRA VFO", r.freqStd);
        if (r.paDevBursts > 0) {
            tw::TextColored(warn, "Phase adjustment bits unlike EN 300 392-2 9.4.4.3.6 in %d of %d bursts", r.paDevBursts, r.n);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("The base station sets the phase adjustment bits differently from equation 9.14.\n"
                                  "This is a coding matter, not modulation accuracy: the bits as sent are used\n"
                                  "for the ideal symbols, so the vector error is not affected.");
            }
        }
        // status of the method
        std::string why;
        if (r.n < etsi_vem::Stats::TARGET) { why += "fewer than 200 bursts; "; }
        if (!gainHeld) { why += "receiver gain not held; "; }
        if (freqRef == 0) { why += "no frequency reference (frequency only); "; }
        if (why.empty()) { tw::TextColored(good, "Measured as in EN 300 394-1 10.1.3"); }
        else {
            why.resize(why.size() - 2);
            tw::TextColored(warn, "Indicative:"); tw::SameTextDisabled("%s", why.c_str());
        }
        if (tw::SameSmallButton(("Reset##_vem_rst_" + id).c_str())) { reset(); }
        return changed;
    }

private:
    static constexpr size_t RING = 1 << 20;

    void start(double off) {
        vfo = sigpath::iqFrontEnd.addVFO(name, FS, 125000.0, off);
        curOffset = off;
        resetOffset = off;
        {
            std::lock_guard<std::mutex> lck(ringMtx);
            ring.assign(RING, std::complex<float>(0.0f, 0.0f));
            wIdx = 0;
        }
        if (!sinkInit) { sink.init(&vfo->out, handler, this); sinkInit = true; }
        else { sink.setInput(&vfo->out); }
        sink.start();
        {
            std::lock_guard<std::mutex> lck(cvMtx);
            stopFlag = false;
        }
        resetReq.store(true);
        worker = std::thread(&TetraVemAnalyzer::work, this);
        running = true;
    }

    static void handler(dsp::complex_t* data, int count, void* ctx) {
        TetraVemAnalyzer* _this = (TetraVemAnalyzer*)ctx;
        std::lock_guard<std::mutex> lck(_this->ringMtx);
        for (int i = 0; i < count; i++) {
            _this->ring[(size_t)(_this->wIdx & (RING - 1))] = std::complex<float>(data[i].re, data[i].im);
            _this->wIdx++;
        }
    }

    void work() {
        etsi_vem::Engine eng;
        const size_t WIN = eng.window(0.3);
        while (true) {
            {
                std::unique_lock<std::mutex> lck(cvMtx);
                if (cv.wait_for(lck, std::chrono::milliseconds(300), [this] { return stopFlag; })) { return; }
            }
            std::vector<std::complex<float>> x;
            uint64_t end;
            {
                std::lock_guard<std::mutex> lck(ringMtx);
                end = wIdx;
                if (end < WIN) { continue; }
                x.resize(WIN);
                for (size_t i = 0; i < WIN; i++) { x[i] = ring[(size_t)((end - WIN + i) & (RING - 1))]; }
            }
            if (resetReq.exchange(false)) { eng.reset(); }
            eng.setCellScrambling(scrReq.load());
            eng.process(x, end - WIN);
            std::lock_guard<std::mutex> lck(resMtx);
            res = eng.stats();
        }
    }

    std::string name = "TETRA modulation accuracy";
    dsp::channel::RxVFO* vfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> sink;
    bool sinkInit = false;
    bool running = false;
    double curOffset = 0.0, resetOffset = 0.0;
    std::mutex ringMtx;
    std::vector<std::complex<float>> ring;
    uint64_t wIdx = 0;
    std::thread worker;
    std::mutex cvMtx;
    std::condition_variable cv;
    bool stopFlag = false;
    std::atomic<bool> resetReq { false };
    std::atomic<uint32_t> scrReq { 0 };
    std::mutex resMtx;
    TetraVemResult res;
};
