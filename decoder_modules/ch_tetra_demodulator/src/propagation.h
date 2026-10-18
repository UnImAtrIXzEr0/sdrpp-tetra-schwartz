#pragma once
// Propagation diagram: local mean level along a drive (50 m averages, see
// propagation_math.h) around the base station - polar plot with north up,
// distance scaled to the farthest point, colour = level.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <imgui.h>
#include <gui/style.h>
#include "tetra_colors.h"
#include "tetra_layout.h"
#include "propagation_math.h"

class TetraPropagationView {
public:
    // settings (saved by the module)
    bool bsValid = false;
    double bsLat = 0.0, bsLon = 0.0;
    int metric = 0;   // 0 level, 1 MER

    prop::Binner bins;

    // 20 samples/s: ask due() first, then feed()
    bool due(double t) const { return t - lastFeed >= 0.05; }
    void feed(double t, bool fix, double lat, double lon, float levelDb, float merDb) {
        if (t - lastFeed < 0.05) { return; }
        lastFeed = t;
        bins.add(t, fix, lat, lon, levelDb, merDb);
        curFix = fix; curLat = lat; curLon = lon;
    }

    // returns true when a setting changed
    bool draw(const std::string& id, float width, bool cal) {
        bool changed = false;
        ImVec4 val = tc::val();
        // base station position
        tw::Text("Base station:");
        ImGui::SetNextItemWidth((std::min)(width * 0.45f, 130.0f * style::uiScale));
        if (ImGui::InputDouble(("##_pp_lat_" + id).c_str(), &bsLat, 0.0, 0.0, "%.6f")) { bsLat = std::clamp(bsLat, -90.0, 90.0); bsValid = true; changed = true; }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Latitude, decimal degrees (north positive)"); }
        ImGui::SameLine();
        ImGui::SetNextItemWidth((std::min)(width * 0.45f, 130.0f * style::uiScale));
        if (ImGui::InputDouble(("##_pp_lon_" + id).c_str(), &bsLon, 0.0, 0.0, "%.6f")) { bsLon = std::clamp(bsLon, -180.0, 180.0); bsValid = true; changed = true; }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Longitude, decimal degrees (east positive)"); }
        if (!curFix) { style::beginDisabled(); }
        if (ImGui::SmallButton(("Use GPS position##_pp_here_" + id).c_str())) { bsLat = curLat; bsLon = curLon; bsValid = true; changed = true; }
        if (!curFix) { style::endDisabled(); }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Take the current GPS position (standing next to the base station)."); }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90.0f * style::uiScale);
        if (ImGui::Combo(("##_pp_metric_" + id).c_str(), &metric, "Level\0MER\0")) { changed = true; }
        if (tw::SameSmallButton(("Clear##_pp_clr_" + id).c_str())) { bins.clear(); }

        tw::Text("Points:"); tw::SameTextColored(val, "%d", (int)bins.points.size());
        tw::SameTextDisabled("(50 m averages, next %.0f m)", bins.pending());
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Each point is the power average of the level over 50 m travelled: the\n"
                              "fast fading (multipath, half a wavelength = 35 cm) is averaged out,\n"
                              "the local mean (shadowing by terrain and buildings) remains.\n"
                              "Standing still does not close a point.");
        }
        if (!bsValid) { tw::TextDisabled("Enter the base station position."); return changed; }
        drawPolar(id, width, cal);
        return changed;
    }

private:
    static ImU32 scaleColor(float x) {   // 0..1, luminance-monotonic (dark blue -> green -> yellow)
        static const float c[5][3] = { { 0.27f, 0.00f, 0.33f }, { 0.23f, 0.32f, 0.55f }, { 0.13f, 0.57f, 0.55f }, { 0.37f, 0.79f, 0.38f }, { 0.99f, 0.91f, 0.14f } };
        x = std::clamp(x, 0.0f, 1.0f) * 4.0f;
        int i = (std::min)(3, (int)x);
        float f = x - i;
        return ImGui::ColorConvertFloat4ToU32(ImVec4(c[i][0] + f * (c[i + 1][0] - c[i][0]), c[i][1] + f * (c[i + 1][1] - c[i][1]), c[i][2] + f * (c[i + 1][2] - c[i][2]), 1.0f));
    }

    void drawPolar(const std::string& id, float width, bool cal) {
        const auto& pts = bins.points;
        // value range and farthest point
        float vmin = 1e9f, vmax = -1e9f;
        double dmax = 0.0;
        for (auto& p : pts) {
            float v = metric == 0 ? p.levelDb : p.merDb;
            if (!std::isnan(v)) { vmin = (std::min)(vmin, v); vmax = (std::max)(vmax, v); }
            dmax = (std::max)(dmax, prop::distanceM(bsLat, bsLon, p.lat, p.lon));
        }
        if (curFix) { dmax = (std::max)(dmax, prop::distanceM(bsLat, bsLon, curLat, curLon)); }
        double step = prop::niceStep(dmax);
        int rings = (std::max)(1, (int)std::ceil((std::max)(dmax, 1.0) / step));
        double full = step * rings;
        if (vmax - vmin < 3.0f) { float m = 0.5f * (vmin + vmax); vmin = m - 1.5f; vmax = m + 1.5f; }

        float side = (std::min)(width, 420.0f * style::uiScale);
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::InvisibleButton(("##_pp_plot_" + id).c_str(), ImVec2(side, side));
        bool hovered = ImGui::IsItemHovered();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 c(p0.x + side / 2, p0.y + side / 2);
        float R = side / 2 - ImGui::GetFontSize() * 1.2f;
        ImU32 grid = ImGui::GetColorU32(ImGuiCol_Border), txt = ImGui::GetColorU32(ImGuiCol_TextDisabled);
        for (int i = 1; i <= rings; i++) {
            float r = R * i / rings;
            dl->AddCircle(c, r, grid, 72, 1.0f);
            char b[32];
            double m = step * i;
            if (full >= 2000.0) { snprintf(b, sizeof(b), "%g km", m / 1000.0); } else { snprintf(b, sizeof(b), "%.0f m", m); }
            dl->AddText(ImVec2(c.x + 3, c.y - r), txt, b);
        }
        const char* nesw[4] = { "N", "E", "S", "W" };
        for (int k = 0; k < 4; k++) {
            float a = k * 1.5707963f;
            ImVec2 d(std::sin(a), -std::cos(a));
            dl->AddLine(ImVec2(c.x - d.x * R, c.y - d.y * R), ImVec2(c.x + d.x * R, c.y + d.y * R), grid, 1.0f);
            ImVec2 ts = ImGui::CalcTextSize(nesw[k]);
            dl->AddText(ImVec2(c.x + d.x * (R + ts.y * 0.7f) - ts.x / 2, c.y + d.y * (R + ts.y * 0.7f) - ts.y / 2), txt, nesw[k]);
        }
        auto toScreen = [&](double lat, double lon) {
            double d = prop::distanceM(bsLat, bsLon, lat, lon) / full * R;
            double b = prop::bearingDeg(bsLat, bsLon, lat, lon) * prop::DEG;
            return ImVec2(c.x + (float)(std::sin(b) * d), c.y - (float)(std::cos(b) * d));
        };
        float dot = (std::max)(2.5f, 3.0f * style::uiScale);
        int hoverIdx = -1;
        float hoverD = 1e9f;
        ImVec2 mouse = ImGui::GetMousePos();
        for (size_t i = 0; i < pts.size(); i++) {
            float v = metric == 0 ? pts[i].levelDb : pts[i].merDb;
            ImVec2 s = toScreen(pts[i].lat, pts[i].lon);
            if (std::isnan(v)) { dl->AddCircle(s, dot, txt, 12, 1.0f); }
            else { dl->AddCircleFilled(s, dot, scaleColor((v - vmin) / (vmax - vmin)), 12); }
            float dd = std::hypot(s.x - mouse.x, s.y - mouse.y);
            if (dd < hoverD) { hoverD = dd; hoverIdx = (int)i; }
        }
        // base station and current position
        dl->AddTriangleFilled(ImVec2(c.x, c.y - dot * 2), ImVec2(c.x - dot * 1.7f, c.y + dot), ImVec2(c.x + dot * 1.7f, c.y + dot), ImGui::GetColorU32(ImGuiCol_Text));
        if (curFix) {
            ImVec2 s = toScreen(curLat, curLon);
            dl->AddCircle(s, dot * 2.2f, ImGui::GetColorU32(ImGuiCol_Text), 16, 1.5f);
        }
        if (hovered && hoverIdx >= 0 && hoverD < dot * 3) {
            const auto& p = pts[hoverIdx];
            ImGui::SetTooltip("%.0f m, %.0f deg\nLevel %.1f %s\nMER %s\n%d samples",
                              prop::distanceM(bsLat, bsLon, p.lat, p.lon), prop::bearingDeg(bsLat, bsLon, p.lat, p.lon),
                              p.levelDb, cal ? "dBm" : "dBFS", std::isnan(p.merDb) ? "--" : (std::to_string((int)std::lround(p.merDb)) + " dB").c_str(), p.samples);
        }
        // colour bar
        if (!pts.empty() && vmin < vmax) {
            ImVec2 b0 = ImGui::GetCursorScreenPos();
            float bw = side * 0.6f, bh = ImGui::GetFontSize() * 0.6f;
            for (int i = 0; i < 32; i++) {
                dl->AddRectFilled(ImVec2(b0.x + bw * i / 32, b0.y), ImVec2(b0.x + bw * (i + 1) / 32 + 0.5f, b0.y + bh), scaleColor(i / 31.0f));
            }
            ImGui::Dummy(ImVec2(bw, bh));
            const char* u = metric == 1 ? "dB MER" : (cal ? "dBm" : "dBFS");
            tw::TextDisabled("%.0f ... %.0f %s (scaled to the points)", vmin, vmax, u);
        }
    }

    double lastFeed = -1.0;
    bool curFix = false;
    double curLat = 0.0, curLon = 0.0;
};
