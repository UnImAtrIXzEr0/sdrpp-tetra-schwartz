#pragma once
// GUI of the trend evaluations: frequency stability (Allan deviation of the
// carrier vs. SYSINFO frequency) and reception quality versus level.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>
#include <imgui.h>
#include <imgui_internal.h>
#include <gui/style.h>
#include "freq_quality.h"
#include "const_view.h"
#include "tetra_colors.h"
#include "tetra_layout.h"

class TetraEvalView {
public:
    float crcLimit = 1.0f;     // % CRC errors that still count as good reception

    // pts: live data (whole store) or the loaded log; refreshed every 2 s
    void update(const std::vector<TetraTrendPoint>& pts, bool force) {
        auto now = std::chrono::steady_clock::now();
        if (!force && std::chrono::duration<double>(now - last).count() < 2.0) { return; }
        last = now;
        std::vector<double> y;
        y.reserve(pts.size());
        ppbT.clear();
        ppbV.clear();
        time_t prev = 0;
        for (auto& p : pts) {
            // gaps in time count as missing values
            if (prev && p.t > prev + 1) { for (time_t t = prev + 1; t < p.t && y.size() < 200000; t++) { y.push_back(NAN); } }
            y.push_back(p.ppb);
            prev = p.t;
            if (!std::isnan(p.ppb)) { ppbT.push_back((double)p.t); ppbV.push_back(p.ppb); }
        }
        adev = tetra_eval::allanDeviation(y, runLen);
        double s = 0.0;
        for (float v : ppbV) { s += v; }
        ppbMean = ppbV.empty() ? NAN : s / ppbV.size();
        quality = tetra_eval::qualityVsLevel(pts, crcLimit);
    }

    void drawStability(const std::string& id, float width, const TetraConstellationView::Palette& pal) {
        tw::TextDisabled("Received carrier vs. SYSINFO frequency (decoder locked).\n"
                         "Contains base station AND SDR error - meaningful with a\n"
                         "GPS-disciplined / reference-locked SDR only.");
        if (ppbV.empty()) { tw::TextDisabled("no data yet (needs lock and SYSINFO)"); return; }
        tw::Text("Mean offset:");
        tw::SameTextColored(tc::val(), "%+.1f ppb", ppbMean);
        tw::SameTextDisabled("(ETSI base station: +-200 ppb)");
        drawSeries(id + "s", width, pal);
        if (adev.empty()) { tw::TextDisabled("Allan deviation: needs at least 10 s without gaps."); return; }
        tw::Text("Allan deviation");
        tw::SameTextDisabled("(%zu s without gaps)", runLen);
        drawAdev(id + "a", width, pal);
    }

    void drawQuality(const std::string& id, float width, const TetraConstellationView::Palette& pal, bool& changed) {
        ImGui::SetNextItemWidth(90.0f * style::uiScale);
        if (ImGui::InputFloat(("CRC limit %##_ev_crc_" + id).c_str(), &crcLimit, 0.5f, 5.0f, "%.1f")) {
            crcLimit = std::clamp(crcLimit, 0.1f, 50.0f);
            changed = true;
            last = {};
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Seconds with more CRC errors than this count as 'not good'."); }
        if (quality.bins.empty()) { tw::TextDisabled("no data yet (needs at least 5 s per 1 dB level step)"); return; }
        const char* unit = quality.calibrated ? "dBm" : "dBFS";
        tw::Text("Reception limit:");
        if (!std::isnan(quality.thresholdLevel)) {
            tw::SameTextColored(tc::val(), "%.0f %s", quality.thresholdLevel, unit);
            tw::SameTextDisabled("(CRC errors <= %.1f %% above)", crcLimit);
        }
        else { tw::SameTextDisabled("not determinable from these data"); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Lowest level from which on every 1 dB step stays within the CRC limit.\n"
                              "Use it as threshold for coverage maps ('Quality' map value).\n"
                              "Calibrate the level (Measurements) for dBm values.");
        }
        drawQualityPlot(id + "q", width, pal, unit);
    }

private:
    void frame(const std::string& id, float width, float h, const TetraConstellationView::Palette& pal, ImVec2& p0, ImRect& pr, float lm) {
        p0 = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton(id.c_str(), ImVec2(width, h));
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), pal.bg);
        dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), pal.border);
        pr = ImRect(ImVec2(p0.x + lm, p0.y + 4), ImVec2(p0.x + width - 6, p0.y + h - 16.0f * style::uiScale));
    }

    void drawSeries(const std::string& id, float width, const TetraConstellationView::Palette& pal) {
        ImVec2 p0;
        ImRect pr;
        frame("##_ev_ser_" + id, width, 110.0f * style::uiScale, pal, p0, pr, 44.0f * style::uiScale);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        double t0 = ppbT.front(), t1 = (std::max)(ppbT.back(), t0 + 1.0);
        float lo = *std::min_element(ppbV.begin(), ppbV.end()), hi = *std::max_element(ppbV.begin(), ppbV.end());
        if (hi - lo < 10.0f) { float c = 0.5f * (hi + lo); lo = c - 5.0f; hi = c + 5.0f; }
        auto Y = [&](float v) { return pr.Max.y - (v - lo) / (hi - lo) * pr.GetHeight(); };
        char buf[32];
        for (int i = 0; i <= 3; i++) {
            float v = lo + (hi - lo) * i / 3;
            dl->AddLine(ImVec2(pr.Min.x, Y(v)), ImVec2(pr.Max.x, Y(v)), pal.grid);
            snprintf(buf, sizeof(buf), "%+.0f", v);
            dl->AddText(ImVec2(p0.x + 3, Y(v) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
        }
        int cols = (std::max)(1, (int)pr.GetWidth());
        std::vector<double> sum(cols, 0.0);
        std::vector<int> n(cols, 0);
        for (size_t i = 0; i < ppbT.size(); i++) {
            int k = std::clamp((int)((ppbT[i] - t0) / (t1 - t0) * cols), 0, cols - 1);
            sum[k] += ppbV[i];
            n[k]++;
        }
        dl->PushClipRect(pr.Min, pr.Max, true);
        ImU32 lc = ImGui::GetColorU32(ImGuiCol_PlotLines);
        ImVec2 prev;
        bool have = false;
        for (int k = 0; k < cols; k++) {
            if (!n[k]) { continue; }
            ImVec2 pt(pr.Min.x + k + 0.5f, Y((float)(sum[k] / n[k])));
            if (have && pt.x - prev.x < 4.0f) { dl->AddLine(prev, pt, lc, 1.3f * style::uiScale); }
            prev = pt;
            have = true;
        }
        dl->PopClipRect();
        dl->AddText(ImVec2(pr.Min.x + 4, pr.Min.y + 2), pal.text, "ppb over time");
        fmtSpan(t1 - t0, buf, sizeof(buf));
        dl->AddText(ImVec2(pr.Max.x - ImGui::CalcTextSize(buf).x, pr.Max.y + 1), pal.text, buf);
    }

    static void fmtSpan(double s, char* b, size_t n) {
        if (s < 120) { snprintf(b, n, "%.0f s", s); }
        else if (s < 7200) { snprintf(b, n, "%.0f min", s / 60); }
        else { snprintf(b, n, "%.1f h", s / 3600); }
    }

    void drawAdev(const std::string& id, float width, const TetraConstellationView::Palette& pal) {
        ImVec2 p0;
        ImRect pr;
        frame("##_ev_adev_" + id, width, 150.0f * style::uiScale, pal, p0, pr, 44.0f * style::uiScale);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        double tmax = adev.back().tau;
        double lt0 = 0.0, lt1 = std::ceil(std::log10((std::max)(tmax, 10.0)));
        double amin = INFINITY, amax = 0.0;
        for (auto& a : adev) { amin = (std::min)(amin, a.adev); amax = (std::max)(amax, a.adev); }
        double la0 = std::floor(std::log10((std::max)(amin, 1e-3))), la1 = std::ceil(std::log10((std::max)(amax, 1e-3)));
        if (la1 - la0 < 1) { la1 = la0 + 1; }
        auto X = [&](double t) { return (float)(pr.Min.x + (std::log10(t) - lt0) / (lt1 - lt0) * pr.GetWidth()); };
        auto Y = [&](double a) { return (float)(pr.Max.y - (std::log10((std::max)(a, 1e-6)) - la0) / (la1 - la0) * pr.GetHeight()); };
        char buf[32];
        for (double e = la0; e <= la1 + 1e-9; e += 1) {
            dl->AddLine(ImVec2(pr.Min.x, Y(std::pow(10, e))), ImVec2(pr.Max.x, Y(std::pow(10, e))), pal.grid);
            snprintf(buf, sizeof(buf), "%g", std::pow(10.0, e));
            dl->AddText(ImVec2(p0.x + 3, Y(std::pow(10, e)) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
        }
        for (double e = lt0; e <= lt1 + 1e-9; e += 1) {
            float x = X(std::pow(10, e));
            dl->AddLine(ImVec2(x, pr.Min.y), ImVec2(x, pr.Max.y), pal.grid);
            snprintf(buf, sizeof(buf), "%g s", std::pow(10.0, e));
            dl->AddText(ImVec2(std::clamp(x - ImGui::CalcTextSize(buf).x * 0.5f, pr.Min.x, pr.Max.x - ImGui::CalcTextSize(buf).x), pr.Max.y + 1), pal.text, buf);
        }
        ImU32 lc = ImGui::GetColorU32(ImGuiCol_PlotLines);
        for (size_t i = 0; i < adev.size(); i++) {
            ImVec2 pt(X(adev[i].tau), Y(adev[i].adev));
            dl->AddCircleFilled(pt, 2.5f * style::uiScale, lc);
            if (i) { dl->AddLine(ImVec2(X(adev[i - 1].tau), Y(adev[i - 1].adev)), pt, lc, 1.3f * style::uiScale); }
        }
        dl->AddText(ImVec2(pr.Min.x + 4, pr.Min.y + 2), pal.text, "ppb vs. tau");
        if (ImGui::IsItemHovered()) {
            std::string t;
            for (auto& a : adev) {
                snprintf(buf, sizeof(buf), "%6.0f s: %.2f ppb\n", a.tau, a.adev);
                t += buf;
            }
            ImGui::SetTooltip("%s", t.c_str());
        }
    }

    void drawQualityPlot(const std::string& id, float width, const TetraConstellationView::Palette& pal, const char* unit) {
        ImVec2 p0;
        ImRect pr;
        frame("##_ev_q_" + id, width, 170.0f * style::uiScale, pal, p0, pr, 44.0f * style::uiScale);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        float x0 = quality.bins.front().level - 0.5f, x1 = quality.bins.back().level + 0.5f;
        if (x1 - x0 < 10.0f) { x1 = x0 + 10.0f; }
        auto X = [&](float l) { return pr.Min.x + (l - x0) / (x1 - x0) * pr.GetWidth(); };
        // CRC fail on a log axis 0.01 .. 100 %
        auto Yc = [&](float pct) { return pr.Max.y - (std::log10(std::clamp(pct, 0.01f, 100.0f)) + 2.0f) / 4.0f * pr.GetHeight(); };
        // MER 0 .. 40 dB on the same height
        auto Ym = [&](float mer) { return pr.Max.y - std::clamp(mer, 0.0f, 40.0f) / 40.0f * pr.GetHeight(); };
        char buf[32];
        const float pcts[5] = { 0.01f, 0.1f, 1.0f, 10.0f, 100.0f };
        for (float p : pcts) {
            dl->AddLine(ImVec2(pr.Min.x, Yc(p)), ImVec2(pr.Max.x, Yc(p)), pal.grid);
            snprintf(buf, sizeof(buf), "%g%%", p);
            dl->AddText(ImVec2(p0.x + 3, Yc(p) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
        }
        float step = (x1 - x0) > 40 ? 10.0f : 5.0f;
        for (float l = std::ceil(x0 / step) * step; l <= x1; l += step) {
            dl->AddLine(ImVec2(X(l), pr.Min.y), ImVec2(X(l), pr.Max.y), pal.grid);
            snprintf(buf, sizeof(buf), "%.0f", l);
            dl->AddText(ImVec2(X(l) - ImGui::CalcTextSize(buf).x * 0.5f, pr.Max.y + 1), pal.text, buf);
        }
        dl->PushClipRect(pr.Min, pr.Max, true);
        ImU32 cc = pal.dotBad, mc = pal.dotGood;
        ImVec2 prevC, prevM;
        bool hc = false, hm = false;
        for (auto& b : quality.bins) {
            ImVec2 pc(X(b.level), Yc(b.crcFail));
            dl->AddCircleFilled(pc, 2.5f * style::uiScale, cc);
            if (hc) { dl->AddLine(prevC, pc, cc, 1.2f); }
            prevC = pc; hc = true;
            if (!std::isnan(b.mer)) {
                ImVec2 pm(X(b.level), Ym(b.mer));
                if (hm) { dl->AddLine(prevM, pm, mc, 1.2f); }
                prevM = pm; hm = true;
            }
        }
        dl->AddLine(ImVec2(pr.Min.x, Yc(crcLimit)), ImVec2(pr.Max.x, Yc(crcLimit)), (cc & 0x00FFFFFF) | 0x70000000);
        if (!std::isnan(quality.thresholdLevel)) {
            float x = X(quality.thresholdLevel);
            dl->AddLine(ImVec2(x, pr.Min.y), ImVec2(x, pr.Max.y), pal.marker, 1.5f);
        }
        dl->PopClipRect();
        snprintf(buf, sizeof(buf), "level %s", unit);
        dl->AddText(ImVec2(pr.Max.x - ImGui::CalcTextSize(buf).x, pr.Min.y + 2), pal.text, buf);
        dl->AddText(ImVec2(pr.Min.x + 4, pr.Min.y + 2), cc, "CRC errors");
        dl->AddText(ImVec2(pr.Min.x + 8 + ImGui::CalcTextSize("CRC errors").x, pr.Min.y + 2), mc, "MER 0-40 dB");
        if (ImGui::IsItemHovered()) {
            float mx = ImGui::GetIO().MousePos.x;
            float l = x0 + (mx - pr.Min.x) / pr.GetWidth() * (x1 - x0);
            const tetra_eval::LevelBin* best = nullptr;
            for (auto& b : quality.bins) { if (!best || std::fabs(b.level - l) < std::fabs(best->level - l)) { best = &b; } }
            if (best) { ImGui::SetTooltip("%.0f %s: CRC errors %.2f %%, MER %.1f dB (%d s)", best->level - 0.5f, unit, best->crcFail, best->mer, best->n); }
        }
    }

    std::chrono::steady_clock::time_point last {};
    std::vector<tetra_eval::AdevPoint> adev;
    size_t runLen = 0;
    std::vector<double> ppbT;
    std::vector<float> ppbV;
    double ppbMean = NAN;
    tetra_eval::QualityCurve quality;
};
