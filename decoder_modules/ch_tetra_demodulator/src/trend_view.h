#pragma once
// Trend view for long-term observation: level, MER, CRC errors, carrier
// offset, timeslot traffic and bursts over 10 min .. 24 h (live) or over a
// loaded CSV log. Several seconds per pixel are combined into min / mean /
// max, so short dropouts stay visible on long time spans.

#include <algorithm>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <imgui.h>
#include <imgui_internal.h>
#include <gui/style.h>
#include "trend.h"
#include "const_view.h"
#include "tetra_colors.h"
#include "tetra_layout.h"

class TetraTrendView {
public:
    enum Panel { P_LEVEL, P_MER, P_CRC, P_CARRIER, P_TS, P_BURSTS, P_COUNT };
    bool show[P_COUNT] = { true, true, true, true, true, false };
    int spanIdx = 1;             // see SPANS
    bool fileMode = false;       // showing a loaded CSV log instead of the live data
    std::string fileName;
    std::vector<TetraTrendPoint> filePts;

    static constexpr int SPAN_N = 6;
    static int spanSeconds(int i) {
        static const int s[SPAN_N] = { 600, 3600, 6 * 3600, 12 * 3600, 24 * 3600, 0 };
        return s[std::clamp(i, 0, SPAN_N - 1)];
    }
    static const char* spanNames() { return "10 min\0" "1 h\0" "6 h\0" "12 h\0" "24 h\0" "All\0"; }

    void invalidate() { cacheSeq = (uint64_t)-1; }

    // draws the panels; store = live data
    void draw(const std::string& id, float width, TetraTrendStore& store, const TetraConstellationView::Palette& pal) {
        const float ph = 78.0f * style::uiScale;
        int nPanels = 0;
        for (int i = 0; i < P_COUNT; i++) { if (show[i]) { nPanels++; } }
        if (nPanels == 0) { tw::TextDisabled("No panel selected."); return; }
        const float lm = 46.0f * style::uiScale;
        int cols = (std::max)(10, (int)(width - lm - 6.0f));

        // rebuild the buckets when new data arrived, at most once per second
        uint64_t seq = fileMode ? 1 : store.getSeq();
        time_t now = time(nullptr);
        if (seq != cacheSeq || cols != cacheCols || spanIdx != cacheSpan || fileMode != cacheFile) {
            rebuild(store, cols, now);
            cacheSeq = seq; cacheCols = cols; cacheSpan = spanIdx; cacheFile = fileMode;
        }
        if (bucketsN == 0) { tw::TextDisabled(fileMode ? "The log contains no data." : "Collecting data (one point per second)..."); return; }

        float h = nPanels * ph + 18.0f * style::uiScale;
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton(id.c_str(), ImVec2(width, h));
        bool hovered = ImGui::IsItemHovered();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, ImVec2(p0.x + width, p0.y + h), pal.bg);
        dl->AddRect(p0, ImVec2(p0.x + width, p0.y + h), pal.border);
        float x0 = p0.x + lm, x1 = p0.x + width - 6.0f;
        int hoverCol = -1;
        if (hovered) {
            float mx = ImGui::GetIO().MousePos.x;
            if (mx >= x0 && mx < x1) { hoverCol = std::clamp((int)((mx - x0) / (x1 - x0) * bucketsN), 0, bucketsN - 1); }
        }

        float y = p0.y + 2.0f;
        for (int pnl = 0; pnl < P_COUNT; pnl++) {
            if (!show[pnl]) { continue; }
            ImRect r(ImVec2(x0, y + 2.0f), ImVec2(x1, y + ph - 4.0f));
            drawPanel(dl, r, pnl, pal);
            y += ph;
        }
        // time axis
        char buf[32];
        int ticks = (std::max)(2, (int)((x1 - x0) / (90.0f * style::uiScale)));
        for (int i = 0; i <= ticks; i++) {
            double t = tStart + (tEnd - tStart) * i / ticks;
            float x = x0 + (x1 - x0) * i / ticks;
            fmtTime((time_t)t, buf, sizeof(buf), (tEnd - tStart) > 86400);
            float tw0 = ImGui::CalcTextSize(buf).x;
            float tx = std::clamp(x - tw0 * 0.5f, p0.x + 2.0f, x1 - tw0);
            dl->AddText(ImVec2(tx, y), pal.text, buf);
        }
        if (hoverCol >= 0) {
            float hx = x0 + (hoverCol + 0.5f) * (x1 - x0) / bucketsN;
            dl->AddLine(ImVec2(hx, p0.y + 2), ImVec2(hx, y), pal.marker);
            tooltip(hoverCol);
        }
    }

    // series stored per bucket
    enum Series { S_LEVEL, S_MER, S_CRC, S_CARRIER, S_TS1, S_TS2, S_TS3, S_TS4, S_BURSTS, S_SYNC, S_COUNT };

private:
    struct Bucket {
        float mn[S_COUNT], mx[S_COUNT], mean[S_COUNT];
        int n[S_COUNT];
    };

    static float value(const TetraTrendPoint& p, int s) {
        switch (s) {
        case S_LEVEL: return p.level;
        case S_MER: return p.mer;
        case S_CRC: return p.crcFail;
        case S_CARRIER: return p.carrierHz;
        case S_TS1: return p.tsTraffic[0];
        case S_TS2: return p.tsTraffic[1];
        case S_TS3: return p.tsTraffic[2];
        case S_TS4: return p.tsTraffic[3];
        case S_BURSTS: return p.bursts;
        case S_SYNC: return p.syncLost;
        }
        return NAN;
    }

    void rebuild(TetraTrendStore& store, int cols, time_t now) {
        int span = spanSeconds(spanIdx);
        std::vector<TetraTrendPoint> pts;
        if (fileMode) {
            pts = filePts;
            if (!pts.empty()) {
                tEnd = (double)pts.back().t + 1.0;
                tStart = span > 0 ? tEnd - span : (double)pts.front().t;
            }
        }
        else {
            tEnd = (double)now + 1.0;
            if (span > 0) {
                tStart = tEnd - span;
                pts = store.copy((time_t)tStart);
            }
            else {
                pts = store.copy(0);
                tStart = pts.empty() ? tEnd - 600.0 : (double)pts.front().t;
            }
        }
        if (tEnd - tStart < 60.0) { tStart = tEnd - 60.0; }
        calibrated = false;
        for (auto& p : pts) { if (p.calibrated) { calibrated = true; } }
        // one bucket per pixel column, but never shorter than one second
        bucketsN = (int)(std::min)((double)cols, tEnd - tStart);
        if (pts.empty()) { bucketsN = 0; return; }
        buckets.assign(bucketsN, Bucket());
        for (auto& b : buckets) {
            for (int s = 0; s < S_COUNT; s++) { b.mn[s] = INFINITY; b.mx[s] = -INFINITY; b.mean[s] = 0.0f; b.n[s] = 0; }
        }
        for (auto& p : pts) {
            double t = (double)p.t;
            if (t < tStart || t >= tEnd) { continue; }
            int k = (int)((t - tStart) / (tEnd - tStart) * bucketsN);
            if (k < 0 || k >= bucketsN) { continue; }
            Bucket& b = buckets[k];
            for (int s = 0; s < S_COUNT; s++) {
                float v = value(p, s);
                if (std::isnan(v)) { continue; }
                b.mn[s] = (std::min)(b.mn[s], v);
                b.mx[s] = (std::max)(b.mx[s], v);
                b.mean[s] += v;
                b.n[s]++;
            }
        }
        for (auto& b : buckets) {
            for (int s = 0; s < S_COUNT; s++) { if (b.n[s]) { b.mean[s] /= b.n[s]; } }
        }
    }

    void range(int s0, int s1, float& lo, float& hi) {
        lo = INFINITY; hi = -INFINITY;
        for (auto& b : buckets) {
            for (int s = s0; s <= s1; s++) {
                if (!b.n[s]) { continue; }
                lo = (std::min)(lo, b.mn[s]);
                hi = (std::max)(hi, b.mx[s]);
            }
        }
    }

    static float niceStep(float span) {
        float raw = span / 3.0f;
        float p = std::pow(10.0f, std::floor(std::log10((std::max)(raw, 1e-6f))));
        float m = raw / p;
        return (m < 1.5f ? 1.0f : m < 3.5f ? 2.0f : m < 7.5f ? 5.0f : 10.0f) * p;
    }

    void drawSeries(ImDrawList* dl, const ImRect& r, int s, float lo, float hi, ImU32 col, bool band) {
        auto Y = [&](float v) { return r.Max.y - (std::clamp(v, lo, hi) - lo) / (hi - lo) * r.GetHeight(); };
        float w = r.GetWidth() / bucketsN;
        if (band) {
            ImU32 bc = (col & 0x00FFFFFF) | 0x40000000;
            for (int k = 0; k < bucketsN; k++) {
                const Bucket& b = buckets[k];
                if (!b.n[s] || b.mx[s] - b.mn[s] <= 0.0f) { continue; }
                float x = r.Min.x + k * w;
                dl->AddRectFilled(ImVec2(x, Y(b.mx[s])), ImVec2(x + (std::max)(w, 1.0f), Y(b.mn[s]) + 1.0f), bc);
            }
        }
        ImVec2 prev;
        bool have = false;
        for (int k = 0; k < bucketsN; k++) {
            const Bucket& b = buckets[k];
            if (!b.n[s]) { have = false; continue; }
            ImVec2 pt(r.Min.x + (k + 0.5f) * w, Y(b.mean[s]));
            if (have) { dl->AddLine(prev, pt, col, 1.3f * style::uiScale); }
            else { dl->AddLine(ImVec2(pt.x - 0.5f, pt.y), ImVec2(pt.x + 0.5f, pt.y), col, 1.3f * style::uiScale); }
            prev = pt;
            have = true;
        }
    }

    static ImU32 tsColor(int i) {
        ImVec4 c[4] = { tc::cyan(), tc::magenta(), tc::orange(), tc::good() };
        return ImGui::ColorConvertFloat4ToU32(c[i & 3]);
    }

    void drawPanel(ImDrawList* dl, const ImRect& r, int pnl, const TetraConstellationView::Palette& pal) {
        int s0, s1;
        const char* title;
        float lo, hi;
        switch (pnl) {
        case P_LEVEL: s0 = s1 = S_LEVEL; title = calibrated ? "Level dBm" : "Level dBFS"; break;
        case P_MER: s0 = s1 = S_MER; title = "MER dB"; break;
        case P_CRC: s0 = s1 = S_CRC; title = "CRC fail %"; break;
        case P_CARRIER: s0 = s1 = S_CARRIER; title = "Carrier Hz"; break;
        case P_TS: s0 = S_TS1; s1 = S_TS4; title = "TS traffic %"; break;
        default: s0 = s1 = S_BURSTS; title = "Bursts/s"; break;
        }
        range(s0, s1, lo, hi);
        bool empty = !(lo <= hi);
        if (pnl == P_TS) { lo = 0.0f; hi = 100.0f; }
        else if (pnl == P_CRC) { lo = 0.0f; hi = empty ? 5.0f : (std::max)(5.0f, hi * 1.1f); }
        else if (pnl == P_BURSTS) { lo = 0.0f; hi = empty ? 80.0f : (std::max)(10.0f, hi * 1.1f); }
        else if (empty) { lo = 0.0f; hi = 1.0f; }
        else {
            float minSpan = pnl == P_CARRIER ? 20.0f : 3.0f;
            if (hi - lo < minSpan) { float c = 0.5f * (hi + lo); lo = c - minSpan / 2; hi = c + minSpan / 2; }
            float pad = 0.08f * (hi - lo);
            lo -= pad; hi += pad;
        }
        // grid and labels
        float step = niceStep(hi - lo);
        char buf[24];
        auto Y = [&](float v) { return r.Max.y - (v - lo) / (hi - lo) * r.GetHeight(); };
        for (float v = std::ceil(lo / step) * step; v <= hi + 1e-6f; v += step) {
            dl->AddLine(ImVec2(r.Min.x, Y(v)), ImVec2(r.Max.x, Y(v)), pal.grid);
            snprintf(buf, sizeof(buf), step < 1.0f ? "%.1f" : "%.0f", v);
            float tw0 = ImGui::CalcTextSize(buf).x;
            dl->AddText(ImVec2(r.Min.x - tw0 - 4.0f, Y(v) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
        }
        dl->AddRect(r.Min, r.Max, pal.grid);
        dl->PushClipRect(r.Min, ImVec2(r.Max.x + 1, r.Max.y + 1), true);
        ImU32 line = ImGui::GetColorU32(ImGuiCol_PlotLines);
        if (pnl == P_TS) {
            for (int i = 0; i < 4; i++) { drawSeries(dl, r, S_TS1 + i, lo, hi, tsColor(i), false); }
        }
        else {
            drawSeries(dl, r, s0, lo, hi, line, true);
        }
        if (pnl == P_CRC || pnl == P_BURSTS) {
            // sync losses as red marks at the top
            float w = r.GetWidth() / bucketsN;
            for (int k = 0; k < bucketsN; k++) {
                if (buckets[k].n[S_SYNC] && buckets[k].mx[S_SYNC] > 0.0f) {
                    float x = r.Min.x + (k + 0.5f) * w;
                    dl->AddLine(ImVec2(x, r.Min.y), ImVec2(x, r.Min.y + 6.0f * style::uiScale), pal.dotBad, 2.0f);
                }
            }
        }
        dl->PopClipRect();
        dl->AddText(ImVec2(r.Min.x + 4.0f, r.Min.y + 1.0f), pal.text, title);
        if (pnl == P_TS) {
            float x = r.Min.x + 8.0f + ImGui::CalcTextSize(title).x;
            for (int i = 0; i < 4; i++) {
                snprintf(buf, sizeof(buf), "TS%d", i + 1);
                dl->AddText(ImVec2(x, r.Min.y + 1.0f), tsColor(i), buf);
                x += ImGui::CalcTextSize(buf).x + 6.0f;
            }
        }
    }

    static void fmtTime(time_t t, char* buf, size_t n, bool withDate) {
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &t);
#else
        localtime_r(&t, &lt);
#endif
        strftime(buf, n, withDate ? "%d.%m. %H:%M" : "%H:%M", &lt);
    }

    void tooltip(int k) {
        const Bucket& b = buckets[k];
        double ta = tStart + (tEnd - tStart) * k / bucketsN;
        double tb = tStart + (tEnd - tStart) * (k + 1) / bucketsN;
        char t0[32], t1[32];
        struct tm lt;
        time_t a = (time_t)ta, e = (time_t)tb;
#ifdef _WIN32
        localtime_s(&lt, &a);
#else
        localtime_r(&a, &lt);
#endif
        strftime(t0, sizeof(t0), "%d.%m.%Y %H:%M:%S", &lt);
#ifdef _WIN32
        localtime_s(&lt, &e);
#else
        localtime_r(&e, &lt);
#endif
        strftime(t1, sizeof(t1), "%H:%M:%S", &lt);
        std::string txt = std::string(t0) + (tb - ta > 1.5 ? std::string(" - ") + t1 : std::string()) + "\n";
        auto add = [&](const char* label, int s, const char* fmt) {
            if (!b.n[s]) { return; }
            char v[3][48];
            snprintf(v[0], sizeof(v[0]), fmt, b.mean[s]);
            txt += std::string(label) + v[0];
            if (b.n[s] > 1 && b.mx[s] > b.mn[s]) {
                snprintf(v[1], sizeof(v[1]), fmt, b.mn[s]);
                snprintf(v[2], sizeof(v[2]), fmt, b.mx[s]);
                txt += std::string("  (min ") + v[1] + ", max " + v[2] + ")";
            }
            txt += "\n";
        };
        add("Level: ", S_LEVEL, calibrated ? "%.1f dBm" : "%.1f dBFS");
        add("MER: ", S_MER, "%.1f dB");
        add("CRC fail: ", S_CRC, "%.2f %%");
        add("Carrier: ", S_CARRIER, "%+.1f Hz");
        add("TS1 traffic: ", S_TS1, "%.0f %%");
        add("TS2 traffic: ", S_TS2, "%.0f %%");
        add("TS3 traffic: ", S_TS3, "%.0f %%");
        add("TS4 traffic: ", S_TS4, "%.0f %%");
        add("Bursts/s: ", S_BURSTS, "%.0f");
        if (b.n[S_SYNC] && b.mx[S_SYNC] > 0.0f) { txt += "Sync lost in this interval\n"; }
        ImGui::SetTooltip("%s", txt.c_str());
    }

    std::vector<Bucket> buckets;
    int bucketsN = 0;
    double tStart = 0.0, tEnd = 0.0;
    bool calibrated = false;
    uint64_t cacheSeq = (uint64_t)-1;
    int cacheCols = -1, cacheSpan = -1;
    bool cacheFile = false;
};
