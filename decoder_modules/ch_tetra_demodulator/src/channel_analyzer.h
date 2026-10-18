#pragma once
// Multipath analysis of the received downlink: least-squares channel
// estimate from the training sequence of every downlink slot, averaged into
// a power delay profile and compared with the profile of an ideal (echo-free)
// signal through the same processing. Shows echoes from about one symbol
// (55 us, 17 km path difference) up to 222 us; shorter delays only widen
// the main path (extra delay spread).
// The tap is a DSP-only VFO of the IQ front end (144 kS/s).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <imgui.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/main_window.h>
#include <signal_path/signal_path.h>
#include "analysis_tap.h"
#include "const_view.h"
#include "tetra_colors.h"
#include "tetra_layout.h"
#include "tetra_sigproc.h"

struct TetraChannelResult {
    bool valid = false;
    int slots = 0;
    float quality = 0.0f;
    tetra_sp::MultipathResult mp;
};

class TetraChannelAnalyzer {
public:
    static constexpr double FS = tetra_sp::FS;

    ~TetraChannelAnalyzer() { stop(); }
    void setTap(TetraAnalysisTap* t) { tap = t; }
    bool isRunning() const { return running; }
    std::string status;

    void loop(bool enable, double freq) {
        bool sdr = gui::mainWindow.sdrIsRunning();
        double center = gui::waterfall.getCenterFrequency();
        double sr = sigpath::iqFrontEnd.getSampleRate();
        double off = freq - center;
        status.clear();
        if (!enable) { stop(); return; }
        if (!sdr) { status = "SDR stopped"; stop(); return; }
        if (!(sr > 0) || std::fabs(off) > TetraAnalysisTap::maxOffset(sr)) { status = "TETRA VFO outside the SDR span"; stop(); return; }
        if (!running) { start(off); }
        else if (std::fabs(off - curOffset) > 0.5) {
            curOffset = off;
            tap->setOffset(off);
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
        tap->unsubscribe(this);
        running = false;
        std::lock_guard<std::mutex> lck(resMtx);
        res = TetraChannelResult();
    }

    void reset() { resetReq.store(true); }
    void setMemorySeconds(int s) { memSec.store(s); }

    TetraChannelResult get() {
        std::lock_guard<std::mutex> lck(resMtx);
        return res;
    }

    // ---- GUI; returns true when a setting changed ----
    bool draw(const std::string& id, float width, const TetraConstellationView::Palette& pal, bool& run, int& memIdx) {
        bool changed = false;
        if (ImGui::Checkbox(("Run multipath analysis##_mp_run_" + id).c_str(), &run)) { changed = true; }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Channel estimate from the training sequences of the downlink bursts of\n"
                              "the TETRA VFO (separate tap). Echoes from about 55 us (17 km detour)\n"
                              "up to 222 us are shown separately; shorter ones widen the main path.\n"
                              "Needs a clean signal (MER above about 15 dB).");
        }
        if (!run) {
            tw::TextDisabled("Echoes (delay, level) and extra delay spread of the\ndownlink, e.g. while driving or at a new site.");
            return changed;
        }
        if (!status.empty()) { tw::TextColored(tc::warn(), "%s", status.c_str()); }
        static const int mems[4] = { 2, 10, 60, 0 };
        ImGui::SetNextItemWidth((std::min)(width * 0.5f, 110.0f * style::uiScale));
        if (ImGui::Combo(("Averaging##_mp_mem_" + id).c_str(), &memIdx, "2 s\0" "10 s\0" "60 s\0" "unlimited\0")) { changed = true; reset(); }
        memIdx = std::clamp(memIdx, 0, 3);
        setMemorySeconds(mems[memIdx]);
        tw::SameNext();
        if (tw::SameButtonOrFull(("Reset##_mp_rst_" + id).c_str())) { reset(); }
        TetraChannelResult r = get();
        if (!r.valid) {
            tw::TextDisabled(r.quality > 0 ? "no training sequences found (decoder locked? MER?)" : "collecting...");
            return changed;
        }
        tw::Text("Slots evaluated:"); tw::SameTextColored(tc::val(), "%d", r.slots);
        tw::Text("Strongest echo:");
        if (!std::isnan(r.mp.echoDelayUs)) {
            tw::SameTextColored(tc::warn(), "%+.0f us (%.1f km detour), %.1f dB", r.mp.echoDelayUs, std::fabs(r.mp.echoDelayUs) * 0.29979, r.mp.echoLevelDb);
        }
        else { tw::SameTextColored(tc::good(), "none above -15 dB"); }
        tw::Text("Extra delay spread:");
        tw::SameTextColored(tc::val(), "%.1f us", r.mp.extraSpreadUs);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("RMS delay spread of the measured profile beyond that of an ideal\n"
                              "signal. TETRA receivers cope with echoes up to about a quarter\n"
                              "symbol (14 us) without loss; larger values reduce the MER.");
        }
        drawProfile(id, width, pal, r);
        return changed;
    }

private:
    static const size_t RING = 1 << 18;

    void start(double off) {
        if (!tap) { return; }
        curOffset = off;
        resetOffset = off;
        {
            std::lock_guard<std::mutex> lck(ringMtx);
            ring.assign(RING, std::complex<float>(0.0f, 0.0f));
            wIdx = 0;
            filter.reset();
        }
        if (!tap->subscribe(this, true, handler, off)) { return; }
        stopFlag = false;
        resetReq.store(true);
        worker = std::thread(&TetraChannelAnalyzer::work, this);
        running = true;
    }

    static void handler(const std::complex<float>* data, int count, void* ctx) {
        TetraChannelAnalyzer* _this = (TetraChannelAnalyzer*)ctx;
        std::lock_guard<std::mutex> lck(_this->ringMtx);
        for (int i = 0; i < count; i++) {
            _this->ring[(size_t)(_this->wIdx & (RING - 1))] = _this->filter.step(data[i]);
            _this->wIdx++;
        }
    }

    void work() {
        // reference: ideal signal through the same processing (computed once)
        if (ideal.empty()) {
            auto syn = tetra_sp::syntheticDownlink(300, 7);
            tetra_sp::ChanFilter f;
            for (auto& v : syn) { v = f.step(v); }
            tetra_sp::ChannelEstimator ce;
            const size_t W = (size_t)(FS * 0.6);
            for (size_t s0 = 0; s0 + W <= syn.size(); s0 += W) {
                std::vector<std::complex<float>> w(syn.begin() + s0, syn.begin() + s0 + W);
                auto e = tetra_sp::slotTiming(w, s0);
                if (e.valid) { ce.accumulate(w, s0, e.phase, e.variant); }
            }
            ideal = ce.profile();
        }
        const size_t WIN = (size_t)(FS * 0.6);
        tetra_sp::ChannelEstimator ce;
        tetra_sp::SlotFoldState slotState;
        uint64_t done = 0;
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
                if (end < WIN || end - done < WIN / 2) { continue; }
                x.resize(WIN);
                for (size_t i = 0; i < WIN; i++) { x[i] = ring[(size_t)((end - WIN + i) & (RING - 1))]; }
            }
            if (resetReq.exchange(false)) { ce.reset(); slotState.reset(); }
            int ms = memSec.load();
            ce.setMemory(ms > 0 ? (int)(ms * 70.6) : 0);   // 70.6 slots per second
            const size_t SW = (size_t)(FS * 0.25);   // short: robust against sample clock offsets
            std::vector<std::complex<float>> xs(x.end() - SW, x.end());
            auto e = tetra_sp::slotTiming(xs, end - SW, &slotState);
            // evaluate only the part not seen before (windows overlap), within
            // the stretch the slot grid was taken from
            if (e.valid) {
                size_t skip = (size_t)(done > end - SW ? done - (end - SW) : 0);
                if (skip < SW) {
                    std::vector<std::complex<float>> y(xs.begin() + skip, xs.end());
                    ce.accumulate(y, end - SW + skip, e.phase, e.variant);
                }
            }
            done = end;
            TetraChannelResult r;
            r.quality = (float)e.quality;
            r.slots = ce.count();
            if (ce.count() >= 20) {
                r.mp = tetra_sp::compareProfiles(ce.profile(), ideal);
                r.valid = r.mp.valid;
            }
            std::lock_guard<std::mutex> lck(resMtx);
            res = r;
        }
    }

    void drawProfile(const std::string& id, float width, const TetraConstellationView::Palette& pal, const TetraChannelResult& r) {
        const float h = 170.0f * style::uiScale, lm = 34.0f * style::uiScale;
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton(("##_mp_prof_" + id).c_str(), ImVec2(width, h));
        bool hovered = ImGui::IsItemHovered();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), pal.bg);
        dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), pal.border);
        ImRect pr(ImVec2(p0.x + lm, p0.y + 4), ImVec2(p0.x + width - 6, p0.y + h - 16.0f * style::uiScale));
        const auto& m = r.mp.measured;
        const auto& id0 = r.mp.ideal;
        const int n = (int)m.size();
        if (n < 2 || (int)id0.size() != n) { return; }
        int pkM = (int)(std::max_element(m.begin(), m.end()) - m.begin());
        int pkI = (int)(std::max_element(id0.begin(), id0.end()) - id0.begin());
        // x axis: delay relative to the main path in us
        double d0 = (0 - pkM) / FS * 1e6, d1 = (n - 1 - pkM) / FS * 1e6;
        auto X = [&](double us) { return (float)(pr.Min.x + (us - d0) / (d1 - d0) * pr.GetWidth()); };
        const float top = 0.0f, bot = -30.0f;
        auto Y = [&](float v) {
            float db = v > 1e-6f ? 10.0f * std::log10(v) : bot;
            return pr.Min.y + (top - std::clamp(db, bot, top)) / (top - bot) * pr.GetHeight();
        };
        char buf[32];
        for (float db = 0; db >= bot; db -= 10) {
            float y = pr.Min.y + (top - db) / (top - bot) * pr.GetHeight();
            dl->AddLine(ImVec2(pr.Min.x, y), ImVec2(pr.Max.x, y), pal.grid);
            snprintf(buf, sizeof(buf), "%.0f", db);
            dl->AddText(ImVec2(p0.x + 3, y - ImGui::GetFontSize() * 0.5f), pal.text, buf);
        }
        for (double us = std::ceil(d0 / 50.0) * 50.0; us <= d1; us += 50.0) {
            dl->AddLine(ImVec2(X(us), pr.Min.y), ImVec2(X(us), pr.Max.y), pal.grid);
            snprintf(buf, sizeof(buf), "%.0f", us);
            dl->AddText(ImVec2(X(us) - ImGui::CalcTextSize(buf).x * 0.5f, pr.Max.y + 1), pal.text, buf);
        }
        dl->AddText(ImVec2(pr.Max.x - ImGui::CalcTextSize("us").x - 2, pr.Max.y + 1), pal.text, "us");
        dl->PushClipRect(pr.Min, pr.Max, true);
        auto curve = [&](const std::vector<float>& v, int shift, ImU32 col, float th) {
            ImVec2 prev;
            bool have = false;
            for (int b = 0; b < n; b++) {
                int k = b - shift;
                if (k < 0 || k >= n) { continue; }
                ImVec2 pt(X((b - pkM) / FS * 1e6), Y(v[k]));
                if (have) { dl->AddLine(prev, pt, col, th); }
                prev = pt;
                have = true;
            }
        };
        curve(id0, pkM - pkI, pal.dotGood, 1.0f);
        curve(m, 0, ImGui::GetColorU32(ImGuiCol_PlotLines), 1.6f * style::uiScale);
        if (!std::isnan(r.mp.echoDelayUs)) {
            float x = X(r.mp.echoDelayUs);
            dl->AddLine(ImVec2(x, pr.Min.y), ImVec2(x, pr.Max.y), pal.dotBad, 1.0f);
        }
        dl->PopClipRect();
        dl->AddText(ImVec2(pr.Min.x + 4, pr.Min.y + 2), pal.text, "measured / ideal (green), dB");
        if (hovered) {
            float mx = ImGui::GetIO().MousePos.x;
            if (mx >= pr.Min.x && mx <= pr.Max.x) {
                double us = d0 + (mx - pr.Min.x) / pr.GetWidth() * (d1 - d0);
                int b = std::clamp((int)std::lround(us * 1e-6 * FS) + pkM, 0, n - 1);
                ImGui::SetTooltip("%+.0f us: %.1f dB (ideal %.1f dB)", us, 10.0 * std::log10((std::max)(m[b], 1e-6f)),
                                  10.0 * std::log10((std::max)(id0[std::clamp(b - (pkM - pkI), 0, n - 1)], 1e-6f)));
            }
        }
    }

    TetraAnalysisTap* tap = nullptr;
    bool running = false;
    double curOffset = 0.0, resetOffset = 0.0;
    std::mutex ringMtx;
    std::vector<std::complex<float>> ring;
    uint64_t wIdx = 0;
    tetra_sp::ChanFilter filter;
    std::thread worker;
    std::mutex cvMtx;
    std::condition_variable cv;
    bool stopFlag = false;
    std::atomic<bool> resetReq { false };
    std::atomic<int> memSec { 10 };
    std::vector<float> ideal;
    std::mutex resMtx;
    TetraChannelResult res;
};
