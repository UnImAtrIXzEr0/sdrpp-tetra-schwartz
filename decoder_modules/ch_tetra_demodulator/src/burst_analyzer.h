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
#include "tetra_sigproc.h"
#include <deque>
#include <ctime>

struct TetraBurstSettings {
    int source = 0;            // 0 uplink from SYSINFO, 1 manual frequency, 2 TETRA VFO frequency
    double manualMHz = 430.0;
    int averages = 20;
    float maskRippleDb = 1.0f; // useful part within +-ripple of the burst power
    float maskOffDbc = -40.0f; // before the ramp-up and after the ramp-down
    float maskRampSym = 5.0f;  // max. ramp time (-20 dBc ... -1 dBc) in symbols
    bool timing = true;        // burst timing against the downlink slot grid (2nd tap on the TETRA VFO)
    bool transients = true;    // spectrum during the ramps
    int floorSpan = 0;         // history of the uplink noise: 0 10 min, 1 1 h, 2 6 h, 3 24 h
};

class TetraBurstAnalyzer {
public:
    static constexpr double FS = TetraBurstDsp::FS;
    static constexpr double SYM = TetraBurstDsp::SYM;
    static constexpr double TAP_BW = 125000.0;   // wide tap: own channel and +-50 kHz

    ~TetraBurstAnalyzer() { stop(); }

    bool isRunning() const { return running; }
    bool hasDlTap() const { return dlvfo != nullptr; }
    std::string status;   // why it is not running (GUI)

    // every GUI frame: run at absolute frequency 'freq' if it is inside the SDR
    // span; dlFreq > 0 adds a sample-aligned tap on the downlink for the timing
    void loop(bool enable, double freq, double dlFreq) {
        bool sdr = gui::mainWindow.sdrIsRunning();
        double center = gui::waterfall.getCenterFrequency();
        double sr = sigpath::iqFrontEnd.getSampleRate();
        double off = freq - center;
        double dlOff = dlFreq - center;
        status.clear();
        if (!enable) { stop(); return; }
        if (freq < 1e6) { status = "no frequency (uplink not known yet - needs SYSINFO)"; stop(); return; }
        if (!sdr) { status = "SDR stopped"; stop(); return; }
        double lim = sr / 2.0 - TAP_BW / 2.0;
        if (!(sr > 0) || std::fabs(off) > lim) {
            char b[200];
            snprintf(b, sizeof(b), "%.4f MHz is outside the usable SDR span (%.4f MHz +- %.2f MHz)", freq / 1e6, center / 1e6, (std::max)(0.0, lim) / 1e6);
            status = b;
            stop();
            return;
        }
        bool wantDl = dlFreq > 1e6 && std::fabs(dlOff) <= lim && std::fabs(dlFreq - freq) > 50000.0;
        if (dlFreq > 1e6 && !wantDl && std::fabs(dlFreq - freq) > 50000.0) { status = "downlink outside the SDR span: no timing measurement"; }
        if (running && wantDl != (dlvfo != nullptr)) { stop(); }
        if (!running) { start(off, wantDl ? dlOff : NAN); }
        else {
            if (std::fabs(off - curOffset) > 0.5) {
                curOffset = off;
                rxvfo->setOffset(off);
                resetReq.store(true);
            }
            if (dlvfo && std::fabs(dlOff - curDlOffset) > 0.5) {
                curDlOffset = dlOff;
                dlvfo->setOffset(dlOff);
            }
        }
        // uplink noise history (once per second)
        time_t now = time(nullptr);
        if (now != lastHist) {
            lastHist = now;
            TetraBurstResult r = get();
            if (!std::isnan(r.gapDbfs)) {
                floorHist.push_back({ now, r.gapDbfs });
                while (floorHist.size() > 24 * 3600) { floorHist.pop_front(); }
            }
        }
    }

    void setAverages(int n) { avgReq.store(std::clamp(n, 1, 500)); }
    void setTransients(bool t) { transReq.store(t); }
    void reset() { resetReq.store(true); }

    TetraBurstResult get() {
        std::lock_guard<std::mutex> lck(resMtx);
        return res;
    }
    float slotQuality() { return dlQuality.load(); }

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
        if (dlvfo) {
            dlSink.stop();
            sigpath::iqFrontEnd.removeVFO(name + " DL");
            dlvfo = nullptr;
        }
        rxvfo = nullptr;
        running = false;
        dlQuality.store(0.0f);
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
        setTransients(st.transients);
        {
            std::string l0 = "Timing vs. downlink##_ba_tim_" + id, l1 = "Ramp spectrum##_ba_tr_" + id;
            tw::CheckRow row;
            row.add(l0.c_str(), &st.timing);
            row.add(l1.c_str(), &st.transients);
            if (row.draw("Also:") >= 0) { changed = true; reset(); }
        }

        TetraBurstResult r = get();
        if (r.state == 0) { tw::TextDisabled("waiting for samples..."); return changed; }
        drawFloor(id, width, pal, r, st, levelCal, changed);
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
        if (st.timing) { drawTiming(id, width, pal, r); }
        if (st.transients) { drawTransients(id, width, r); }
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

    void drawFloor(const std::string& id, float width, const TetraConstellationView::Palette& pal, const TetraBurstResult& r,
                   TetraBurstSettings& st, float levelCal, bool& changed) {
        bool cal = levelCal != 0.0f;
        tw::Text("Noise between bursts:");
        if (!std::isnan(r.gapDbfs)) { tw::SameTextColored(tc::val(), cal ? "%.1f dBm" : "%.1f dBFS", r.gapDbfs + levelCal); }
        else { tw::SameTextDisabled("--"); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Power in the 25 kHz channel while nobody transmits (1 ms away from all\n"
                              "bursts). On the uplink of a repeater this is its receive noise and\n"
                              "interference - a rise means something disturbs the repeater's receiver.\n"
                              "Limited by the noise of the SDR; set an alarm under Alarms.");
        }
        if (floorHist.size() < 2) { return; }
        if (ImGui::TreeNode(("Noise history##_ba_fh_" + id).c_str())) {
            ImGui::SetNextItemWidth(90.0f * style::uiScale);
            if (ImGui::Combo(("Span##_ba_fhs_" + id).c_str(), &st.floorSpan, "10 min\0" "1 h\0" "6 h\0" "24 h\0")) { changed = true; }
            static const int spans[4] = { 600, 3600, 6 * 3600, 24 * 3600 };
            int span = spans[std::clamp(st.floorSpan, 0, 3)];
            const float h = 110.0f * style::uiScale, lm = 40.0f * style::uiScale;
            ImVec2 p0 = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton(("##_ba_fhp_" + id).c_str(), ImVec2(width, h));
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), pal.bg);
            dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), pal.border);
            ImRect pr(ImVec2(p0.x + lm, p0.y + 4), ImVec2(p0.x + width - 4, p0.y + h - 16.0f * style::uiScale));
            time_t now = time(nullptr);
            double t0 = (double)now - span, t1 = (double)now;
            int cols = (std::max)(1, (int)pr.GetWidth());
            std::vector<float> mx(cols, -INFINITY), mean(cols, 0.0f);
            std::vector<int> n(cols, 0);
            for (auto it = floorHist.rbegin(); it != floorHist.rend() && (double)it->first >= t0; ++it) {
                int k = (int)(((double)it->first - t0) / (t1 - t0) * cols);
                if (k < 0 || k >= cols) { continue; }
                mx[k] = (std::max)(mx[k], it->second);
                mean[k] += it->second;
                n[k]++;
            }
            float lo = INFINITY, hi = -INFINITY;
            for (int k = 0; k < cols; k++) { if (n[k]) { mean[k] /= n[k]; lo = (std::min)(lo, mean[k]); hi = (std::max)(hi, mx[k]); } }
            if (lo <= hi) {
                lo = std::floor((lo + levelCal - 2.0f) / 5.0f) * 5.0f - levelCal;
                hi = std::ceil((hi + levelCal + 2.0f) / 5.0f) * 5.0f - levelCal;
                auto Y = [&](float v) { return pr.Max.y - (v - lo) / (hi - lo) * pr.GetHeight(); };
                char buf[32];
                float step = (hi - lo) > 20.0f ? 10.0f : 5.0f;
                for (float v = lo; v <= hi + 0.01f; v += step) {
                    dl->AddLine(ImVec2(pr.Min.x, Y(v)), ImVec2(pr.Max.x, Y(v)), pal.grid);
                    snprintf(buf, sizeof(buf), "%.0f", v + levelCal);
                    dl->AddText(ImVec2(p0.x + 3, Y(v) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
                }
                for (int i = 0; i <= 4; i++) {
                    time_t t = (time_t)(t0 + (t1 - t0) * i / 4);
                    struct tm lt;
#ifdef _WIN32
                    localtime_s(&lt, &t);
#else
                    localtime_r(&t, &lt);
#endif
                    strftime(buf, sizeof(buf), "%H:%M", &lt);
                    float tx = pr.Min.x + pr.GetWidth() * i / 4 - ImGui::CalcTextSize(buf).x * 0.5f;
                    dl->AddText(ImVec2(std::clamp(tx, pr.Min.x, pr.Max.x - ImGui::CalcTextSize(buf).x), pr.Max.y + 1), pal.text, buf);
                }
                dl->PushClipRect(pr.Min, pr.Max, true);
                ImU32 lc = ImGui::GetColorU32(ImGuiCol_PlotLines);
                ImU32 mc = (pal.dotBad & 0x00FFFFFF) | 0xA0000000;
                ImVec2 prev;
                bool have = false;
                for (int k = 0; k < cols; k++) {
                    if (!n[k]) { continue; }
                    float x = pr.Min.x + k + 0.5f;
                    if (mx[k] > mean[k] + 0.2f) { dl->AddLine(ImVec2(x, Y(mean[k])), ImVec2(x, Y(mx[k])), mc); }
                    ImVec2 pt(x, Y(mean[k]));
                    if (have && pt.x - prev.x < 4.0f) { dl->AddLine(prev, pt, lc, 1.3f * style::uiScale); }
                    prev = pt;
                    have = true;
                }
                dl->PopClipRect();
            }
            tw::TextDisabled("Line: mean, red: maximum per pixel.");
            ImGui::TreePop();
        }
    }

    void drawTiming(const std::string& id, float width, const TetraConstellationView::Palette& pal, const TetraBurstResult& r) {
        if (!ImGui::TreeNodeEx(("Burst timing vs. downlink##_ba_tn_" + id).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) { return; }
        if (!hasDlTap()) {
            tw::TextDisabled("Needs the TETRA VFO on the downlink of the same cell\n(inside the SDR span, more than 50 kHz from the burst frequency).");
            ImGui::TreePop();
            return;
        }
        float q = slotQuality();
        tw::Text("Downlink slot grid:");
        if (q > 3.5f) { tw::SameTextColored(tc::good(), "found (quality %.1f)", q); }
        else { tw::SameTextColored(tc::warn(), "not found yet"); }
        if (r.timingValid) {
            tw::Text("Rising edge after slot start:");
            tw::SameTextColored(tc::val(), "%.2f sym (%.0f us)", r.timingMeanSym, r.timingMeanSym / SYM * 1e6);
            tw::SameTextDisabled("+- %.2f sym, %d bursts", r.timingStdSym, r.timingN);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("-3 dB point of the rising edge, measured from the start of the uplink\n"
                                  "slot as given by the received downlink (uplink slots are two slots\n"
                                  "after the downlink ones). Compare mobiles with each other: the value\n"
                                  "also contains the path differences base station -> mobile -> receiver\n"
                                  "vs. base station -> receiver (3.3 us per km), so put the receiver close\n"
                                  "to the mobile for an absolute check.");
            }
            // scatter: offset over time
            const float h = 120.0f * style::uiScale, lm = 40.0f * style::uiScale;
            ImVec2 p0 = ImGui::GetCursorScreenPos();
            ImGui::InvisibleButton(("##_ba_tsc_" + id).c_str(), ImVec2(width, h));
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), pal.bg);
            dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), pal.border);
            ImRect pr(ImVec2(p0.x + lm, p0.y + 4), ImVec2(p0.x + width - 4, p0.y + h - 4));
            float c = r.timingMeanSym, half = (std::max)(2.0f, (std::min)(64.0f, 6.0f * r.timingStdSym));
            float lo = c - half, hi = c + half;
            auto Y = [&](float v) { return pr.Max.y - (v - lo) / (hi - lo) * pr.GetHeight(); };
            char buf[32];
            for (int i = 0; i <= 4; i++) {
                float v = lo + (hi - lo) * i / 4;
                dl->AddLine(ImVec2(pr.Min.x, Y(v)), ImVec2(pr.Max.x, Y(v)), pal.grid);
                snprintf(buf, sizeof(buf), "%.1f", v);
                dl->AddText(ImVec2(p0.x + 3, Y(v) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
            }
            if (!r.timing.empty()) {
                double ta = r.timing.front().t, tb = r.timing.back().t;
                if (tb - ta < 1.0) { tb = ta + 1.0; }
                dl->PushClipRect(pr.Min, pr.Max, true);
                for (auto& pt : r.timing) {
                    float v = pt.offsetSym;
                    while (v - c > 127.5f) { v -= 255.0f; }
                    while (v - c < -127.5f) { v += 255.0f; }
                    float x = pr.Min.x + (float)((pt.t - ta) / (tb - ta)) * pr.GetWidth();
                    dl->AddCircleFilled(ImVec2(x, Y(v)), 2.0f * style::uiScale, pal.dotGood);
                }
                dl->PopClipRect();
            }
            dl->AddText(ImVec2(pr.Min.x + 4, pr.Min.y + 2), pal.text, "symbols, last bursts");
        }
        else { tw::TextDisabled("no bursts with known slot grid yet"); }
        ImGui::TreePop();
    }

    void drawTransients(const std::string& id, float width, const TetraBurstResult& r) {
        if (!ImGui::TreeNodeEx(("Ramp spectrum (switching transients)##_ba_trn_" + id).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) { return; }
        if (!r.transValid) { tw::TextDisabled("needs full-slot bursts"); ImGui::TreePop(); return; }
        tw::TextDisabled("Channel power (TETRA filter) in 1.8 ms windows, dBc\nrelative to the own channel in the middle of the burst.");
        ImGuiTableFlags tf = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX;
        if (ImGui::BeginTable(("##_ba_trt_" + id).c_str(), 6, tf, ImVec2(width, ImGui::GetTextLineHeightWithSpacing() * 5.6f))) {
            tw::CellScope cells;
            ImGui::TableSetupScrollFreeze(1, 1);
            ImGui::TableSetupColumn("Window");
            const char* hdr[5] = { "-50 kHz", "-25 kHz", "own", "+25 kHz", "+50 kHz" };
            for (int o = 0; o < 5; o++) { ImGui::TableSetupColumn(hdr[o]); }
            ImGui::TableHeadersRow();
            const char* rows[3] = { "ramp up", "middle", "ramp down" };
            for (int w = 0; w < 3; w++) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(rows[w]);
                for (int o = 0; o < 5; o++) {
                    ImGui::TableSetColumnIndex(o + 1);
                    float v = r.transDbc[w][o];
                    bool nearFloor = !std::isnan(r.transFloorDbc[o]) && v < r.transFloorDbc[o] + 3.0f;
                    if (o == 2) { ImGui::TextColored(tc::lgrey(), "%.1f", v); }
                    else if (nearFloor) { ImGui::TextDisabled("%.1f", v); }
                    else { ImGui::TextColored(w == 1 ? tc::val() : (v > r.transDbc[1][o] + 6.0f ? tc::warn() : tc::val()), "%.1f", v); }
                }
            }
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextDisabled("floor");
            for (int o = 0; o < 5; o++) {
                ImGui::TableSetColumnIndex(o + 1);
                if (!std::isnan(r.transFloorDbc[o])) { ImGui::TextDisabled("%.1f", r.transFloorDbc[o]); } else { ImGui::TextDisabled("--"); }
            }
            ImGui::EndTable();
        }
        tw::TextDisabled("Orange: ramp window more than 6 dB above the middle (splatter).\nGrey: at the measurement floor. %d bursts.", r.transN);
        ImGui::TreePop();
    }

    void start(double offset, double dlOffset) {
        bool withDl = !std::isnan(dlOffset);
        if (withDl) {
            auto v = sigpath::iqFrontEnd.addVFOs({ name, name + " DL" }, FS, TAP_BW, { offset, dlOffset });
            if (v.size() != 2) { return; }
            rxvfo = v[0];
            dlvfo = v[1];
            curDlOffset = dlOffset;
        }
        else {
            rxvfo = sigpath::iqFrontEnd.addVFO(name, FS, TAP_BW, offset);
            if (!rxvfo) { return; }
        }
        curOffset = offset;
        {
            std::lock_guard<std::mutex> lck(ringMtx);
            ringRaw.assign(RING, std::complex<float>(0.0f, 0.0f));
            ringFilt.assign(RING, std::complex<float>(0.0f, 0.0f));
            ringDl.assign(withDl ? RING : 0, std::complex<float>(0.0f, 0.0f));
            wIdx = 0;
            dlIdx = 0;
            ulFilter.reset();
            dlFilter.reset();
        }
        if (!sinkInit) { sink.init(&rxvfo->out, handler, this); sinkInit = true; }
        else { sink.setInput(&rxvfo->out); }
        if (withDl) {
            if (!dlSinkInit) { dlSink.init(&dlvfo->out, dlHandler, this); dlSinkInit = true; }
            else { dlSink.setInput(&dlvfo->out); }
            dlSink.start();
        }
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
            std::complex<float> v(data[i].re, data[i].im);
            size_t k = (size_t)(_this->wIdx & (RING - 1));
            _this->ringRaw[k] = v;
            _this->ringFilt[k] = _this->ulFilter.step(v);
            _this->wIdx++;
        }
    }

    static void dlHandler(dsp::complex_t* data, int count, void* ctx) {
        TetraBurstAnalyzer* _this = (TetraBurstAnalyzer*)ctx;
        std::lock_guard<std::mutex> lck(_this->ringMtx);
        if (_this->ringDl.empty()) { return; }
        for (int i = 0; i < count; i++) {
            _this->ringDl[(size_t)(_this->dlIdx & (RING - 1))] = _this->dlFilter.step(std::complex<float>(data[i].re, data[i].im));
            _this->dlIdx++;
        }
    }

    void work() {
        const size_t WIN = (size_t)(FS * 0.6);
        TetraBurstDsp dsp;
        tetra_sp::SlotFoldState slotState;
        auto t0 = std::chrono::steady_clock::now();
        double phase = NAN;
        int sinceTiming = 100;
        while (true) {
            {
                std::unique_lock<std::mutex> lck(cvMtx);
                if (cv.wait_for(lck, std::chrono::milliseconds(150), [this] { return stopFlag; })) { return; }
            }
            std::vector<std::complex<float>> x, raw, dlx;
            uint64_t end;
            bool withDl;
            {
                std::lock_guard<std::mutex> lck(ringMtx);
                withDl = !ringDl.empty();
                end = withDl ? (std::min)(wIdx, dlIdx) : wIdx;
                if (end < WIN) { continue; }
                x.resize(WIN);
                raw.resize(WIN);
                uint64_t s0 = end - WIN;
                for (size_t i = 0; i < WIN; i++) {
                    size_t k = (size_t)((s0 + i) & (RING - 1));
                    x[i] = ringFilt[k];
                    raw[i] = ringRaw[k];
                }
                if (withDl && ++sinceTiming >= 6) {   // slot grid about once per second
                    dlx.resize(WIN);
                    for (size_t i = 0; i < WIN; i++) { dlx[i] = ringDl[(size_t)((s0 + i) & (RING - 1))]; }
                }
            }
            if (resetReq.exchange(false)) { dsp.reset(); slotState.reset(); }
            if (!dlx.empty()) {
                sinceTiming = 0;
                // newest 0.25 s: short enough that a sample clock offset of
                // the SDR does not smear the correlation peak
                const size_t SW = (size_t)(FS * 0.25);
                std::vector<std::complex<float>> dls(dlx.end() - SW, dlx.end());
                tetra_sp::SlotEstimate e = tetra_sp::slotTiming(dls, end - SW, &slotState);
                dlQuality.store((float)e.quality);
                phase = e.valid ? e.phase : NAN;
            }
            double nowS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            dsp.process(x, end - WIN, nowS, avgReq.load(), transReq.load() ? &raw : nullptr, ulFilter.delay(), withDl ? phase : NAN);
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
    dsp::channel::RxVFO* dlvfo = nullptr;
    dsp::sink::Handler<dsp::complex_t> sink, dlSink;
    bool sinkInit = false, dlSinkInit = false;
    bool running = false;
    double curOffset = 0.0, curDlOffset = 0.0;

    std::mutex ringMtx;
    std::vector<std::complex<float>> ringRaw, ringFilt, ringDl;
    uint64_t wIdx = 0, dlIdx = 0;
    tetra_sp::ChanFilter ulFilter, dlFilter;
    std::atomic<float> dlQuality { 0.0f };
    std::atomic<bool> transReq { true };
    std::deque<std::pair<time_t, float>> floorHist;
    time_t lastHist = 0;

    std::thread worker;
    std::mutex cvMtx;
    std::condition_variable cv;
    bool stopFlag = false;

    std::atomic<int> avgReq { 20 };
    std::atomic<bool> resetReq { false };
    std::mutex resMtx;
    TetraBurstResult res;
};
