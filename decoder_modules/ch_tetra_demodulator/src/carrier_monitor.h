#pragma once
// Multi-carrier monitor: up to 8 further TETRA carriers (neighbour cells,
// other repeaters) inside the SDR bandwidth are demodulated at the same time
// as the main carrier - level, MER, carrier offset and symbol sync, no
// decoding (the osmo-tetra decoder exists once per program). Each carrier is a
// DSP-only VFO of the IQ front end with its own pi/4-DQPSK demodulator.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <deque>
#include <memory>
#include <string>
#include <vector>
#include <imgui.h>
#include <gui/gui.h>
#include <gui/style.h>
#include <gui/main_window.h>
#include <signal_path/signal_path.h>
#include <dsp/sink/handler_sink.h>
#include "dsp/pi4dqpsk.h"
#include "dsp/dqpsk_sym_extr.h"
#include "const_view.h"
#include "tetra_colors.h"
#include "tetra_layout.h"

struct TetraDemodParams {
    double sampleRate = 72000.0, bandwidth = 45000.0;
    int rrcTaps = 65;
    double rrcAlpha = 0.35, agcRate = 0.05, costasBw = 0.04, fllBw = 0.02, omega = 0.0, mu = 0.0, relLim = 0.05;
};

class TetraCarrierMonitor {
public:
    static const int MAX_CARRIERS = 8;

    struct Point {
        time_t t;
        float level, mer;
    };

    struct Carrier {
        std::string label;
        double freqHz = 0.0;
        std::string vfoName;
        dsp::channel::RxVFO* vfo = nullptr;
        dsp::demod::PI4DQPSK demod;
        dsp::DQPSKSymbolExtractor extr;
        dsp::sink::Handler<uint8_t> sink;
        bool inited = false;
        bool running = false;
        bool inSpan = false;
        int syncSeconds = 0;
        // latest values (once per second)
        bool valid = false;
        float level = -200.0f, mer = 0.0f, carrierHz = 0.0f;
        bool sync = false;
        std::deque<Point> hist;   // 24 h
    };

    ~TetraCarrierMonitor() { stopAll(); stopLog(); }

    void setParams(const TetraDemodParams& p) { params = p; }
    void setBaseName(const std::string& n) { baseName = n; }

    int count() const { return (int)carriers.size(); }
    const Carrier& at(int i) const { return *carriers[i]; }

    bool add(const std::string& label, double freqHz) {
        if ((int)carriers.size() >= MAX_CARRIERS || freqHz < 1e6) { return false; }
        for (auto& c : carriers) { if (std::fabs(c->freqHz - freqHz) < 100.0) { return false; } }
        auto c = std::make_unique<Carrier>();
        c->label = label.empty() ? fmtMHz(freqHz) : label;
        c->freqHz = freqHz;
        c->vfoName = baseName + " carrier " + std::to_string(nextId++);
        carriers.push_back(std::move(c));
        changed = true;
        return true;
    }
    void remove(int i) {
        if (i < 0 || i >= (int)carriers.size()) { return; }
        stop(*carriers[i]);
        carriers.erase(carriers.begin() + i);
        changed = true;
    }
    bool hasFreq(double f) const {
        for (auto& c : carriers) { if (std::fabs(c->freqHz - f) < 100.0) { return true; } }
        return false;
    }
    // true once after the list was changed (to save the configuration)
    bool takeChanged() { bool c = changed; changed = false; return c; }

    void stopAll() {
        for (auto& c : carriers) { stop(*c); }
    }

    // Called every GUI frame: starts / retunes / stops the VFOs and takes the
    // values once per second
    void update(bool enabled) {
        bool sdr = gui::mainWindow.sdrIsRunning();
        double center = gui::waterfall.getCenterFrequency();
        double sr = sigpath::iqFrontEnd.getSampleRate();
        for (auto& cp : carriers) {
            Carrier& c = *cp;
            double off = c.freqHz - center;
            c.inSpan = sr > 0 && std::fabs(off) < sr / 2.0 - params.bandwidth / 2.0 - 5000.0;
            if (enabled && sdr && c.inSpan) {
                if (!c.running) { start(c, off); }
                else if (c.vfo) { c.vfo->setOffset(off); }
            }
            else if (c.running) {
                stop(c);
            }
        }
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - lastSample).count() < 1.0) { return; }
        lastSample = now;
        time_t t = time(nullptr);
        for (auto& cp : carriers) {
            Carrier& c = *cp;
            if (!c.running) { c.valid = false; c.syncSeconds = 0; continue; }
            c.valid = true;
            c.level = c.demod.getLevelDbfs();
            c.mer = c.extr.getMerDb();
            c.carrierHz = c.demod.getCarrierOffsetHz();
            c.sync = c.extr.sync;
            c.syncSeconds = c.sync ? c.syncSeconds + 1 : 0;
            c.demod.setTracking(c.syncSeconds >= 2);
            c.hist.push_back({ t, c.level, c.sync ? c.mer : NAN });
            while (c.hist.size() > 24 * 3600) { c.hist.pop_front(); }
            writeLog(c, t);
        }
        if (log) { fflush(log); }
    }

    // ---- CSV log (next to the measurement log) ----
    void startLog(const std::string& path, float levelCal) {
        stopLog();
        log = fopen(path.c_str(), "w");
        logCal = levelCal;
        if (log) { fprintf(log, "time;label;freq_hz;level_dbfs;level_dbm;mer_db;carrier_offset_hz;sync\n"); }
    }
    void stopLog() {
        if (log) { fclose(log); log = nullptr; }
    }

    // ---- GUI ----
    // mainLevel/mainMer: values of the main carrier for comparison (NaN = none)
    void draw(const std::string& id, float width, const TetraConstellationView::Palette& pal, double mainFreq, float mainLevel, float mainMer,
              bool mainSync, float levelCal, const std::vector<std::pair<std::string, double>>& neighbours) {
        ImVec4 val = tc::val(), good = tc::good(), warn = tc::warn(), grey = tc::grey();
        bool cal = levelCal != 0.0f;
        // add a carrier
        ImGui::SetNextItemWidth((std::min)(width * 0.45f, 140.0f * style::uiScale));
        ImGui::InputDouble(("MHz##_cm_f_" + id).c_str(), &addMHz, 0.0125, 0.1, "%.4f");
        tw::SameNext();
        bool full = (int)carriers.size() >= MAX_CARRIERS;
        if (full) { ImGui::BeginDisabled(); }
        if (tw::SameButtonOrFull(("Add carrier##_cm_add_" + id).c_str())) {
            add("", addMHz * 1e6);
        }
        int newN = 0;
        for (auto& n : neighbours) { if (!hasFreq(n.second) && std::fabs(n.second - mainFreq) > 100.0) { newN++; } }
        if (newN > 0) {
            std::string b = "Add neighbour cells (" + std::to_string(newN) + ")##_cm_nb_" + id;
            if (ImGui::Button(b.c_str())) {
                for (auto& n : neighbours) {
                    if (!hasFreq(n.second) && std::fabs(n.second - mainFreq) > 100.0) { add(n.first, n.second); }
                }
            }
        }
        if (full) { ImGui::EndDisabled(); }

        ImGuiTableFlags tf = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX;
        float rowH = ImGui::GetTextLineHeightWithSpacing();
        float th = rowH * ((float)carriers.size() + 2.4f);
        if (ImGui::BeginTable(("##_cm_tab_" + id).c_str(), 8, tf, ImVec2(width, th))) {
            tw::CellScope cells;
            ImGui::TableSetupScrollFreeze(1, 1);
            ImGui::TableSetupColumn("Carrier");
            ImGui::TableSetupColumn("MHz");
            ImGui::TableSetupColumn(cal ? "dBm" : "dBFS");
            ImGui::TableSetupColumn("vs. main");
            ImGui::TableSetupColumn("MER");
            ImGui::TableSetupColumn("Offset");
            ImGui::TableSetupColumn("Sync");
            ImGui::TableSetupColumn("");
            ImGui::TableHeadersRow();
            // main carrier
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted("Main (TETRA VFO)");
            ImGui::TableSetColumnIndex(1);
            if (mainFreq > 1e6) { ImGui::TextColored(val, "%.4f", mainFreq / 1e6); }
            ImGui::TableSetColumnIndex(2);
            if (!std::isnan(mainLevel)) { ImGui::TextColored(val, "%.1f", mainLevel + levelCal); }
            ImGui::TableSetColumnIndex(4);
            if (mainSync && !std::isnan(mainMer)) { ImGui::TextColored(val, "%.1f dB", mainMer); } else { ImGui::TextDisabled("--"); }
            ImGui::TableSetColumnIndex(6);
            ImGui::TextColored(mainSync ? good : grey, mainSync ? "yes" : "no");
            int del = -1;
            for (int i = 0; i < (int)carriers.size(); i++) {
                Carrier& c = *carriers[i];
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextColored(lineColorV(i), "%s", c.label.c_str());
                ImGui::TableSetColumnIndex(1);
                ImGui::TextColored(val, "%.4f", c.freqHz / 1e6);
                if (!c.inSpan) {
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextColored(warn, "outside SDR span");
                }
                else if (c.valid) {
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextColored(val, "%.1f", c.level + levelCal);
                    ImGui::TableSetColumnIndex(3);
                    if (!std::isnan(mainLevel)) { ImGui::Text("%+.1f dB", c.level - mainLevel); }
                    ImGui::TableSetColumnIndex(4);
                    if (c.sync) { ImGui::TextColored(val, "%.1f dB", c.mer); } else { ImGui::TextDisabled("--"); }
                    ImGui::TableSetColumnIndex(5);
                    if (c.sync) { ImGui::Text("%+.0f Hz", c.carrierHz); } else { ImGui::TextDisabled("--"); }
                    ImGui::TableSetColumnIndex(6);
                    ImGui::TextColored(c.sync ? good : grey, c.sync ? "yes" : "no");
                }
                else {
                    ImGui::TableSetColumnIndex(2);
                    ImGui::TextDisabled("SDR stopped");
                }
                ImGui::TableSetColumnIndex(7);
                if (ImGui::SmallButton(("x##_cm_del_" + std::to_string(i) + id).c_str())) { del = i; }
                if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Remove this carrier"); }
            }
            ImGui::EndTable();
            if (del >= 0) { remove(del); }
        }
        if (carriers.empty()) {
            tw::TextDisabled("Add carriers inside the SDR bandwidth (e.g. neighbour cells from\n"
                             "the network information) to compare level and MER over time.");
            return;
        }
        // level history of all carriers
        ImGui::SetNextItemWidth(90.0f * style::uiScale);
        ImGui::Combo(("Level history##_cm_span_" + id).c_str(), &spanIdx, "10 min\0" "1 h\0" "6 h\0" "24 h\0");
        drawHistory(id, width, pal, cal ? levelCal : 0.0f);
        tw::TextDisabled("Each carrier needs about as much CPU as one more VFO.");
    }

private:
    static std::string fmtMHz(double f) {
        char b[32];
        snprintf(b, sizeof(b), "%.4f MHz", f / 1e6);
        return b;
    }
    static ImVec4 lineColorV(int i) {
        ImVec4 c[8] = { tc::cyan(), tc::magenta(), tc::orange(), tc::good(), tc::bad(), tc::warn(), tc::lgrey(), tc::val() };
        return c[i & 7];
    }

    void start(Carrier& c, double off) {
        c.vfo = sigpath::iqFrontEnd.addVFO(c.vfoName, params.sampleRate, params.bandwidth, off);
        if (!c.vfo) { return; }
        if (!c.inited) {
            c.demod.init(&c.vfo->out, 18000, params.sampleRate, params.rrcTaps, params.rrcAlpha, params.agcRate, params.costasBw, params.fllBw, params.omega, params.mu, params.relLim);
            c.extr.init(&c.demod.out);
            c.sink.init(&c.extr.out, discard, nullptr);
            c.inited = true;
        }
        else {
            c.demod.setInput(&c.vfo->out);
        }
        c.sink.start();
        c.extr.start();
        c.demod.start();
        c.running = true;
        c.syncSeconds = 0;
    }

    void stop(Carrier& c) {
        if (!c.running) { return; }
        c.demod.stop();
        c.extr.stop();
        c.sink.stop();
        sigpath::iqFrontEnd.removeVFO(c.vfoName);
        c.vfo = nullptr;
        c.running = false;
        c.valid = false;
    }

    static void discard(uint8_t* data, int count, void* ctx) {}

    void writeLog(const Carrier& c, time_t t) {
        if (!log) { return; }
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &t);
#else
        localtime_r(&t, &lt);
#endif
        char ts[32];
        strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &lt);
        std::string label = c.label;
        for (auto& ch : label) { if (ch == ';') { ch = ','; } }
        fprintf(log, "%s;%s;%.0f;%.1f;", ts, label.c_str(), c.freqHz, c.level);
        if (logCal != 0.0f) { fprintf(log, "%.1f;", c.level + logCal); } else { fprintf(log, ";"); }
        if (c.sync) { fprintf(log, "%.2f;%.1f;1\n", c.mer, c.carrierHz); } else { fprintf(log, ";;0\n"); }
    }

    void drawHistory(const std::string& id, float width, const TetraConstellationView::Palette& pal, float cal) {
        static const int spans[4] = { 600, 3600, 6 * 3600, 24 * 3600 };
        int span = spans[std::clamp(spanIdx, 0, 3)];
        const float h = 120.0f * style::uiScale, lm = 40.0f * style::uiScale;
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton(("##_cm_hist_" + id).c_str(), ImVec2(width, h));
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), pal.bg);
        dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), pal.border);
        ImRect r(ImVec2(p0.x + lm, p0.y + 4), ImVec2(p0.x + width - 4, p0.y + h - 16.0f * style::uiScale));
        int cols = (std::max)(1, (int)r.GetWidth());
        time_t now = time(nullptr);
        double t0 = (double)now - span, t1 = (double)now;
        // column means, rebuilt once per second (24 h = 86400 points per carrier)
        if (now != cacheT || cols != cacheCols || span != cacheSpan || carriers.size() != cacheMean.size()) {
            cacheT = now; cacheCols = cols; cacheSpan = span;
            cacheMean.assign(carriers.size(), std::vector<float>(cols, NAN));
            for (size_t i = 0; i < carriers.size(); i++) {
                std::vector<double> sum(cols, 0.0);
                std::vector<int> n(cols, 0);
                auto& h = carriers[i]->hist;
                for (auto it = h.rbegin(); it != h.rend() && (double)it->t >= t0; ++it) {
                    int k = (int)(((double)it->t - t0) / (t1 - t0) * cols);
                    if (k < 0 || k >= cols) { continue; }
                    sum[k] += it->level;
                    n[k]++;
                }
                for (int k = 0; k < cols; k++) { if (n[k]) { cacheMean[i][k] = (float)(sum[k] / n[k]); } }
            }
        }
        float lo = INFINITY, hi = -INFINITY;
        for (auto& v : cacheMean) {
            for (float x : v) { if (!std::isnan(x)) { lo = (std::min)(lo, x); hi = (std::max)(hi, x); } }
        }
        if (!(lo <= hi)) { dl->AddText(ImVec2(r.Min.x + 4, r.Min.y + 4), pal.text, "no data yet"); return; }
        lo = std::floor((lo + cal) / 10.0f) * 10.0f - cal;
        hi = std::ceil((hi + cal) / 10.0f) * 10.0f - cal;
        if (hi - lo < 10.0f) { hi = lo + 10.0f; }
        auto X = [&](double t) { return (float)(r.Min.x + (t - t0) / (t1 - t0) * r.GetWidth()); };
        auto Y = [&](float v) { return r.Max.y - (v - lo) / (hi - lo) * r.GetHeight(); };
        char buf[32];
        float step = (hi - lo) > 40.0f ? 20.0f : 10.0f;
        for (float v = lo; v <= hi + 0.01f; v += step) {
            dl->AddLine(ImVec2(r.Min.x, Y(v)), ImVec2(r.Max.x, Y(v)), pal.grid);
            snprintf(buf, sizeof(buf), "%.0f", v + cal);
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
            float x = X((double)t) - ImGui::CalcTextSize(buf).x * 0.5f;
            dl->AddText(ImVec2(std::clamp(x, r.Min.x, r.Max.x - ImGui::CalcTextSize(buf).x), r.Max.y + 1), pal.text, buf);
        }
        dl->PushClipRect(r.Min, r.Max, true);
        for (size_t i = 0; i < cacheMean.size(); i++) {
            ImU32 col = ImGui::ColorConvertFloat4ToU32(lineColorV((int)i));
            ImVec2 prev;
            bool have = false;
            for (int k = 0; k < cols; k++) {
                float v = cacheMean[i][k];
                if (std::isnan(v)) { continue; }
                ImVec2 pt(r.Min.x + k + 0.5f, Y(v));
                if (have && pt.x - prev.x < 4.0f) { dl->AddLine(prev, pt, col, 1.3f * style::uiScale); }
                else { dl->AddLine(ImVec2(pt.x - 0.5f, pt.y), ImVec2(pt.x + 0.5f, pt.y), col, 1.3f * style::uiScale); }
                prev = pt;
                have = true;
            }
        }
        dl->PopClipRect();
    }

    TetraDemodParams params;
    std::string baseName = "TETRA";
    std::vector<std::unique_ptr<Carrier>> carriers;
    int nextId = 1;
    bool changed = false;
    double addMHz = 439.0;
    int spanIdx = 0;
    std::chrono::steady_clock::time_point lastSample;
    FILE* log = nullptr;
    float logCal = 0.0f;
    std::vector<std::vector<float>> cacheMean;
    time_t cacheT = 0;
    int cacheCols = 0, cacheSpan = 0;
};
