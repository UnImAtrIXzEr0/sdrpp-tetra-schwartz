#pragma once
// Burst power analysis of TETRA mobile stations (uplink) or of any other
// burst transmitter: power versus time through a 25 kHz channel filter,
// burst detection, averaged burst profile with ramp-up / ramp-down time,
// burst length, ripple of the useful part and power outside the burst,
// compared with a user-set time mask.
// The tap is a DSP-only VFO of the IQ front end (144 kS/s, 8 samples per
// symbol); the DSP thread only copies samples, the analysis runs in a worker.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
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
#include "const_view.h"
#include "tetra_colors.h"
#include "tetra_layout.h"
#include "burst_dsp.h"

struct TetraBurstSettings {
    int source = 0;            // 0 uplink from SYSINFO, 1 manual frequency, 2 TETRA VFO frequency
    double manualMHz = 430.0;
    int averages = 20;
    float maskRippleDb = 1.0f; // useful part within +-ripple of the burst power
    float maskOffDbc = -40.0f; // before the ramp-up and after the ramp-down
    float maskRampSym = 5.0f;  // max. ramp time (-20 dBc ... -1 dBc) in symbols
};

class TetraBurstAnalyzer {
public:
    static constexpr double FS = TetraBurstDsp::FS;
    static constexpr double SYM = TetraBurstDsp::SYM;

    ~TetraBurstAnalyzer() { stop(); }

    bool isRunning() const { return running; }
    std::string status;   // why it is not running (GUI)

    // every GUI frame: run at absolute frequency 'freq' if it is inside the SDR span
    void loop(bool enable, double freq) {
        bool sdr = gui::mainWindow.sdrIsRunning();
        double center = gui::waterfall.getCenterFrequency();
        double sr = sigpath::iqFrontEnd.getSampleRate();
        double off = freq - center;
        status.clear();
        if (!enable) { stop(); return; }
        if (freq < 1e6) { status = "no frequency (uplink not known yet - needs SYSINFO)"; stop(); return; }
        if (!sdr) { status = "SDR stopped"; stop(); return; }
        if (!(sr > 0) || std::fabs(off) > sr / 2.0 - 20000.0) {
            char b[160];
            snprintf(b, sizeof(b), "%.4f MHz is outside the SDR span (%.4f MHz +- %.2f MHz)", freq / 1e6, center / 1e6, sr / 2e6);
            status = b;
            stop();
            return;
        }
        if (!running) { start(off); }
        else if (std::fabs(off - curOffset) > 0.5) {
            curOffset = off;
            rxvfo->setOffset(off);
            resetReq.store(true);
        }
    }

    void setAverages(int n) { avgReq.store(std::clamp(n, 1, 500)); }
    void reset() { resetReq.store(true); }

    TetraBurstResult get() {
        std::lock_guard<std::mutex> lck(resMtx);
        return res;
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
        rxvfo = nullptr;
        running = false;
        std::lock_guard<std::mutex> lck(resMtx);
        res = TetraBurstResult();
    }

    void setName(const std::string& n) { name = n; }

    // ---- GUI ----
    // returns true when a setting was changed (to be saved)
    bool draw(const std::string& id, float width, const TetraConstellationView::Palette& pal, TetraBurstSettings& st,
              double ulFreq, double vfoFreq, float levelCal, bool& run) {
        bool changed = false;
        ImVec4 val = tc::val(), good = tc::good(), bad = tc::bad(), warn = tc::warn();
        if (ImGui::Checkbox(("Run burst analysis##_ba_run_" + id).c_str(), &run)) { changed = true; }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Power versus time of TETRA bursts (mobile stations on the uplink,\n"
                              "or a base station in discontinuous mode) through a 25 kHz filter.\n"
                              "Separate tap, not shown on the waterfall. The frequency must be\n"
                              "inside the SDR bandwidth.");
        }
        const char* srcNames = "Uplink (SYSINFO)\0Manual frequency\0TETRA VFO\0";
        ImGui::SetNextItemWidth((std::min)(width, 170.0f * style::uiScale));
        if (ImGui::Combo(("##_ba_src_" + id).c_str(), &st.source, srcNames)) { changed = true; reset(); }
        if (st.source == 1) {
            tw::SameNext();
            ImGui::SetNextItemWidth((std::min)(width, 120.0f * style::uiScale));
            if (ImGui::InputDouble(("MHz##_ba_mhz_" + id).c_str(), &st.manualMHz, 0.0125, 0.1, "%.4f")) { changed = true; reset(); }
        }
        double f = st.source == 0 ? ulFreq : st.source == 1 ? st.manualMHz * 1e6 : vfoFreq;
        if (f > 1e6) { tw::Text("Frequency:"); tw::SameTextColored(val, "%.4f MHz", f / 1e6); }
        if (!run) {
            tw::TextDisabled("Ramp-up/down, burst length, ripple and power outside\nthe burst, averaged over the last bursts, with a time mask.");
            return changed;
        }
        if (!status.empty()) { tw::TextColored(warn, "%s", status.c_str()); }
        ImGui::SetNextItemWidth((std::min)(width * 0.5f, 110.0f * style::uiScale));
        if (ImGui::InputInt(("Averages##_ba_avg_" + id).c_str(), &st.averages, 5, 20)) { st.averages = std::clamp(st.averages, 1, 500); changed = true; }
        tw::SameNext();
        if (tw::SameButtonOrFull(("Reset##_ba_rst_" + id).c_str())) { reset(); }
        setAverages(st.averages);

        TetraBurstResult r = get();
        if (r.state == 0) { tw::TextDisabled("waiting for samples..."); return changed; }
        bool cal = levelCal != 0.0f;
        tw::Text("Floor:"); tw::SameTextColored(val, cal ? "%.1f dBm" : "%.1f dBFS", r.floorDbfs + levelCal);
        tw::SameText(" peak"); tw::SameTextColored(val, cal ? "%.1f dBm" : "%.1f dBFS", r.topDbfs + levelCal);
        if (r.state == 1) {
            tw::TextColored(warn, r.continuous ? "No bursts: continuous carrier or noise only" : "No bursts detected (waiting for a transmission)");
        }
        tw::Text("Bursts:"); tw::SameTextColored(val, "%llu full slot", (unsigned long long)r.nFull);
        tw::SameTextColored(val, " %llu half slot", (unsigned long long)r.nHalf);
        tw::SameText(" ("); tw::SameTextColored(val, "%.1f/s", r.burstsPerSec); tw::SameText(")");
        if (r.profileDbc.empty()) { return changed; }

        auto mark = [&](bool ok) { return ok ? good : bad; };
        tw::Text("Burst power:"); tw::SameTextColored(val, cal ? "%.1f dBm" : "%.1f dBFS", r.powerDbfs + levelCal);
        tw::SameTextDisabled(r.fromHalf ? "(avg. of %d half-slot bursts)" : "(avg. of %d bursts)", r.averaged);
        const double nominal = r.fromHalf ? 103.0 : 231.0;   // control / normal uplink burst, symbols
        tw::Text("Length (-3 dB):");
        if (!std::isnan(r.lengthSym)) {
            tw::SameTextColored(val, "%.1f sym = %.2f ms", r.lengthSym, r.lengthSym / SYM * 1e3);
            tw::SameTextDisabled("(nominal %.0f sym)", nominal);
        }
        tw::Text("Ramp up:");
        if (!std::isnan(r.riseSym)) { tw::SameTextColored(mark(r.riseSym <= st.maskRampSym), "%.1f sym (%.0f us)", r.riseSym, r.riseSym / SYM * 1e6); }
        else { tw::SameTextDisabled("--"); }
        tw::SameText(" down:");
        if (!std::isnan(r.fallSym)) { tw::SameTextColored(mark(r.fallSym <= st.maskRampSym), "%.1f sym (%.0f us)", r.fallSym, r.fallSym / SYM * 1e6); }
        else { tw::SameTextDisabled("--"); }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Time from -20 dBc to -1 dBc (ramp up) and from -1 dBc to -20 dBc (ramp down)."); }
        tw::Text("Ripple (useful part):");
        if (!std::isnan(r.ripplePosDb)) {
            bool ok = r.ripplePosDb <= st.maskRippleDb && -r.rippleNegDb <= st.maskRippleDb;
            tw::SameTextColored(mark(ok), "%+.2f / %+.2f dB", r.ripplePosDb, r.rippleNegDb);
        }
        tw::Text("Outside the burst:");
        if (!std::isnan(r.offBeforeDbc)) {
            bool limited = !std::isnan(r.floorDbc) && r.offBeforeDbc < r.floorDbc + 3.0f;
            tw::SameTextColored(limited ? val : mark(r.offBeforeDbc <= st.maskOffDbc), "%.1f dBc before", r.offBeforeDbc);
            if (!std::isnan(r.offAfterDbc)) { tw::SameTextColored(limited ? val : mark(r.offAfterDbc <= st.maskOffDbc), "%.1f dBc after", r.offAfterDbc); }
            if (limited) { tw::SameTextDisabled("(at the floor)"); }
        }
        drawProfile(id, width, pal, r, st);
        if (ImGui::TreeNode(("Time mask##_ba_mask_" + id).c_str())) {
            ImGui::SetNextItemWidth(90.0f * style::uiScale);
            if (ImGui::InputFloat(("Ripple +- dB##_ba_mr_" + id).c_str(), &st.maskRippleDb, 0.1f, 0.5f, "%.1f")) { st.maskRippleDb = std::clamp(st.maskRippleDb, 0.1f, 10.0f); changed = true; }
            ImGui::SetNextItemWidth(90.0f * style::uiScale);
            if (ImGui::InputFloat(("Outside dBc##_ba_mo_" + id).c_str(), &st.maskOffDbc, 1.0f, 5.0f, "%.0f")) { st.maskOffDbc = std::clamp(st.maskOffDbc, -90.0f, -10.0f); changed = true; }
            ImGui::SetNextItemWidth(90.0f * style::uiScale);
            if (ImGui::InputFloat(("Ramp max. sym##_ba_mrs_" + id).c_str(), &st.maskRampSym, 0.5f, 2.0f, "%.1f")) { st.maskRampSym = std::clamp(st.maskRampSym, 0.5f, 40.0f); changed = true; }
            tw::TextDisabled("User mask. The defaults are guide values only -\nplease check them against the standard.");
            ImGui::TreePop();
        }
        return changed;
    }

private:
    static const size_t RING = 1 << 18;   // 1.8 s

    void start(double offset) {
        rxvfo = sigpath::iqFrontEnd.addVFO(name, FS, 25000.0, offset);
        if (!rxvfo) { return; }
        curOffset = offset;
        {
            std::lock_guard<std::mutex> lck(ringMtx);
            ring.assign(RING, std::complex<float>(0.0f, 0.0f));
            wIdx = 0;
        }
        if (!sinkInit) { sink.init(&rxvfo->out, handler, this); sinkInit = true; }
        else { sink.setInput(&rxvfo->out); }
        sink.start();
        stopFlag = false;
        resetReq.store(true);
        worker = std::thread(&TetraBurstAnalyzer::work, this);
        running = true;
    }

    static void handler(dsp::complex_t* data, int count, void* ctx) {
        TetraBurstAnalyzer* _this = (TetraBurstAnalyzer*)ctx;
        std::lock_guard<std::mutex> lck(_this->ringMtx);
        for (int i = 0; i < count; i++) {
            _this->ring[(size_t)(_this->wIdx & (RING - 1))] = std::complex<float>(data[i].re, data[i].im);
            _this->wIdx++;
        }
    }

    void work() {
        const size_t WIN = (size_t)(FS * 0.6);
        TetraBurstDsp dsp;
        auto t0 = std::chrono::steady_clock::now();
        while (true) {
            {
                std::unique_lock<std::mutex> lck(cvMtx);
                if (cv.wait_for(lck, std::chrono::milliseconds(150), [this] { return stopFlag; })) { return; }
            }
            std::vector<std::complex<float>> x;
            uint64_t end;
            {
                std::lock_guard<std::mutex> lck(ringMtx);
                end = wIdx;
                if (end < WIN) { continue; }
                x.resize(WIN);
                uint64_t s0 = end - WIN;
                for (size_t i = 0; i < WIN; i++) { x[i] = ring[(size_t)((s0 + i) & (RING - 1))]; }
            }
            if (resetReq.exchange(false)) { dsp.reset(); }
            double nowS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            dsp.process(x, end - WIN, nowS, avgReq.load());
            std::lock_guard<std::mutex> lck(resMtx);
            res = dsp.result();
        }
    }

    void drawProfile(const std::string& id, float width, const TetraConstellationView::Palette& pal, const TetraBurstResult& r, const TetraBurstSettings& st) {
        const float h = 200.0f * style::uiScale, lm = 34.0f * style::uiScale;
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton(("##_ba_prof_" + id).c_str(), ImVec2(width, h));
        bool hovered = ImGui::IsItemHovered();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), pal.bg);
        dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), pal.border);
        ImRect pr(ImVec2(p0.x + lm, p0.y + 4), ImVec2(p0.x + width - 6, p0.y + h - 16.0f * style::uiScale));
        const double symPerS = SYM;
        double xs0 = -r.preS * symPerS, xs1 = (r.profileDbc.size() / r.fs - r.preS) * symPerS;
        const float top = 5.0f, bot = -70.0f;
        auto X = [&](double s) { return (float)(pr.Min.x + (s - xs0) / (xs1 - xs0) * pr.GetWidth()); };
        auto Y = [&](float db) { return pr.Min.y + (top - std::clamp(db, bot, top)) / (top - bot) * pr.GetHeight(); };
        char buf[32];
        for (float db = 0.0f; db >= bot; db -= 10.0f) {
            dl->AddLine(ImVec2(pr.Min.x, Y(db)), ImVec2(pr.Max.x, Y(db)), pal.grid);
            snprintf(buf, sizeof(buf), "%.0f", db);
            dl->AddText(ImVec2(p0.x + 3, Y(db) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
        }
        for (double s = 0; s <= xs1; s += 50.0) {
            dl->AddLine(ImVec2(X(s), pr.Min.y), ImVec2(X(s), pr.Max.y), pal.grid);
            snprintf(buf, sizeof(buf), "%.0f", s);
            dl->AddText(ImVec2(X(s) - ImGui::CalcTextSize(buf).x * 0.5f, pr.Max.y + 1), pal.text, buf);
        }
        dl->AddText(ImVec2(pr.Max.x - ImGui::CalcTextSize("symbols").x - 2, pr.Max.y + 1), pal.text, "symbols");
        // mask: useful part within +-ripple, outside the ramps below 'off'
        double len = std::isnan(r.lengthSym) ? (r.fromHalf ? 103.0 : 231.0) : r.lengthSym;
        double ramp = st.maskRampSym;
        ImU32 mc = pal.dotBad;
        dl->PushClipRect(pr.Min, pr.Max, true);
        dl->AddLine(ImVec2(X(xs0), Y(st.maskOffDbc)), ImVec2(X(-ramp), Y(st.maskOffDbc)), mc, 1.5f);
        dl->AddLine(ImVec2(X(len + ramp), Y(st.maskOffDbc)), ImVec2(X(xs1), Y(st.maskOffDbc)), mc, 1.5f);
        dl->AddLine(ImVec2(X(ramp), Y(st.maskRippleDb)), ImVec2(X(len - ramp), Y(st.maskRippleDb)), mc, 1.5f);
        dl->AddLine(ImVec2(X(ramp), Y(-st.maskRippleDb)), ImVec2(X(len - ramp), Y(-st.maskRippleDb)), mc, 1.5f);
        // measurement floor
        if (!std::isnan(r.floorDbc)) {
            dl->AddLine(ImVec2(pr.Min.x, Y(r.floorDbc)), ImVec2(pr.Max.x, Y(r.floorDbc)), IM_COL32(140, 140, 140, 160));
        }
        // profile: max per pixel column keeps the ramps visible
        int cols = (std::max)(1, (int)pr.GetWidth());
        const int n = (int)r.profileDbc.size();
        ImU32 lc = ImGui::GetColorU32(ImGuiCol_PlotLines);
        ImVec2 prev;
        bool have = false;
        for (int c = 0; c < cols; c++) {
            int ka = (int)((double)c / cols * n), kb = (std::max)(ka + 1, (int)((double)(c + 1) / cols * n));
            float mx = -200.0f;
            for (int k = ka; k < kb && k < n; k++) { mx = (std::max)(mx, r.profileDbc[k]); }
            ImVec2 pt(pr.Min.x + c, Y(mx));
            if (have) { dl->AddLine(prev, pt, lc, 1.4f * style::uiScale); }
            prev = pt;
            have = true;
        }
        dl->PopClipRect();
        if (hovered) {
            float mx = ImGui::GetIO().MousePos.x;
            if (mx >= pr.Min.x && mx <= pr.Max.x && n > 0) {
                double s = xs0 + (mx - pr.Min.x) / pr.GetWidth() * (xs1 - xs0);
                int k = std::clamp((int)((s / symPerS + r.preS) * r.fs), 0, n - 1);
                dl->AddLine(ImVec2(mx, pr.Min.y), ImVec2(mx, pr.Max.y), pal.marker);
                ImGui::SetTooltip("%.1f symbols (%.0f us): %.1f dBc", s, s / symPerS * 1e6, r.profileDbc[k]);
            }
        }
    }

    std::string name = "TETRA bursts";
    dsp::channel::RxVFO* rxvfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> sink;
    bool sinkInit = false;
    bool running = false;
    double curOffset = 0.0;

    std::mutex ringMtx;
    std::vector<std::complex<float>> ring;
    uint64_t wIdx = 0;

    std::thread worker;
    std::mutex cvMtx;
    std::condition_variable cv;
    bool stopFlag = false;

    std::atomic<int> avgReq { 20 };
    std::atomic<bool> resetReq { false };
    std::mutex resMtx;
    TetraBurstResult res;
};
