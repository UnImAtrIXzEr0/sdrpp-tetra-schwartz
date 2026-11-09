#pragma once
// Map of the drive test: the drive track on a plane in metres (local
// equirectangular projection around the view centre - accurate on
// drive-test distances), zoom with the mouse wheel, pan by dragging, scale
// bar, north arrow, legend, the cell site with distance rings and the
// current position. Colours and ranges as in the KMZ/KML export
// (coverage_scale.h). Map tiles can later be drawn underneath.

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <string>
#include <vector>
#include <imgui.h>
#include <gui/style.h>
#include "coverage_scale.h"
#include "drive_track.h"
#include "map_tiles.h"
#include "tetra_colors.h"
#include "tetra_layout.h"
#include "ui_text.h"

namespace drivemap {
    constexpr double PI = 3.14159265358979323846;
    constexpr double M_PER_DEG = 6371008.8 * PI / 180.0;   // mean earth radius

    // 1-2-5 steps: the largest nice value <= v
    inline double niceStep(double v) {
        if (v <= 0) { return 1; }
        double e = std::pow(10.0, std::floor(std::log10(v)));
        double f = v / e;
        return (f >= 5 ? 5 : f >= 2 ? 2 : 1) * e;
    }

    inline std::string distText(double m) {
        char b[32];
        if (m >= 1000) { snprintf(b, sizeof(b), "%g km", m / 1000.0); }
        else { snprintf(b, sizeof(b), "%g m", m); }
        return b;
    }

    // great-circle distance (haversine), metres
    inline double distanceM(double lat1, double lon1, double lat2, double lon2) {
        double p1 = lat1 * PI / 180, p2 = lat2 * PI / 180, dp = p2 - p1, dl = (lon2 - lon1) * PI / 180;
        double a = std::sin(dp / 2) * std::sin(dp / 2) + std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
        return 2 * 6371008.8 * std::asin((std::min)(1.0, std::sqrt(a)));
    }
}

class DriveMapView {
public:
    // settings (saved by the module)
    int metric = 0;               // cov::Metric, shared with the export
    bool follow = true;           // keep the current position in the centre
    bool siteValid = false;       // cell site
    double siteLat = 0, siteLon = 0;
    float crcLimit = 1.0f;

    TileLayer* tiles = nullptr;   // map underneath (nullptr or inactive: grid only)
    std::string attribution;      // of the map provider, always shown on the map

    bool picking = false;         // the next click on the map sets the cell site
    bool siteChanged = false;     // set when the site was picked on the map

    void requestFit() { fitPending = true; }

    // show an area (e.g. a saved map): centre and the scale that fits it into the last drawn size
    void showArea(double la0, double lo0, double la1, double lo1) {
        cLat = (la0 + la1) * 0.5;
        cLon = (lo0 + lo1) * 0.5;
        double ex = (lo1 - lo0) * drivemap::M_PER_DEG * std::cos(cLat * drivemap::PI / 180), ey = (la1 - la0) * drivemap::M_PER_DEG;
        mpp = std::clamp((std::max)(ex / (std::max)(50.0, lastW), ey / (std::max)(50.0, lastH)), 0.5, 20000.0);
        viewSet = true;
        follow = false;
    }

    // view centre and scale (for tests and the tile layer)
    double centerLat() const { return cLat; }
    double centerLon() const { return cLon; }
    double metersPerPixel() const { return mpp; }

    // geographic <-> pixel offset from the view centre
    void project(double lat, double lon, double& x, double& y) const {
        x = (lon - cLon) * drivemap::M_PER_DEG * std::cos(cLat * drivemap::PI / 180) / mpp;
        y = -(lat - cLat) * drivemap::M_PER_DEG / mpp;
    }
    void unproject(double x, double y, double& lat, double& lon) const {
        lat = cLat - y * mpp / drivemap::M_PER_DEG;
        lon = cLon + x * mpp / (drivemap::M_PER_DEG * std::cos(cLat * drivemap::PI / 180));
    }

    // Draws the map into a box of 'size'; pts: track to show, cur*: current fix
    void draw(const char* id, ImVec2 size, const std::vector<DrivePoint>& pts, bool curValid, double curLat, double curLon, float curCourse) {
        size.x = (std::max)(size.x, 50.0f);
        size.y = (std::max)(size.y, 50.0f);
        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImVec2 p1(p0.x + size.x, p0.y + size.y);
        lastW = size.x;
        lastH = size.y;
        ImGui::InvisibleButton(id, size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        bool hovered = ImGui::IsItemHovered();
        ImGuiIO& io = ImGui::GetIO();
        double cx = p0.x + size.x * 0.5, cy = p0.y + size.y * 0.5;

        // view: fit on request or when nothing was shown yet, follow the position
        bool haveData = !pts.empty() || curValid || siteValid;
        if (haveData && (fitPending || !viewSet)) { fit(pts, curValid, curLat, curLon, size); fitPending = false; viewSet = true; }
        if (follow && curValid && !ImGui::IsItemActive()) { cLat = curLat; cLon = curLon; }

        // interaction
        if (hovered && io.MouseWheel != 0.0f) {
            double mx = io.MousePos.x - cx, my = io.MousePos.y - cy, la, lo;
            unproject(mx, my, la, lo);
            mpp = std::clamp(mpp * std::pow(1.25, -io.MouseWheel), 0.2, 20000.0);
            if (!(follow && curValid)) {
                // keep the point under the mouse in place (when following: zoom about the position)
                double nx, ny;
                project(la, lo, nx, ny);
                unproject(nx - mx, ny - my, cLat, cLon);
            }
        }
        if (ImGui::IsItemActive() && ImGui::IsMouseDragging(ImGuiMouseButton_Left, 2.0f)) {
            ImVec2 d = io.MouseDelta;
            if (d.x != 0.0f || d.y != 0.0f) {
                unproject(-d.x, -d.y, cLat, cLon);
                follow = false;
                dragged = true;
            }
        }
        if (ImGui::IsItemDeactivated()) {
            if (!dragged && picking && hovered) {
                unproject(io.MousePos.x - cx, io.MousePos.y - cy, siteLat, siteLon);
                siteValid = true;
                siteChanged = true;
                picking = false;
            }
            dragged = false;
        }
        if (picking && (ImGui::IsItemClicked(ImGuiMouseButton_Right) || ImGui::IsKeyPressed(ImGuiKey_Escape))) { picking = false; }
        if (hovered && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left) && !picking) { fitPending = true; follow = false; }

        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->PushClipRect(p0, p1, true);
        // between window and frame background: not glaring in light themes
        ImVec4 wb = ImGui::GetStyleColorVec4(ImGuiCol_WindowBg), fb = ImGui::GetStyleColorVec4(ImGuiCol_FrameBg);
        dl->AddRectFilled(p0, p1, ImGui::ColorConvertFloat4ToU32(ImVec4((wb.x + fb.x) * 0.5f, (wb.y + fb.y) * 0.5f, (wb.z + fb.z) * 0.5f, 1.0f)));
        float us = style::uiScale;
        ImU32 textCol = ImGui::GetColorU32(ImGuiCol_Text);
        ImU32 dimCol = ImGui::GetColorU32(ImGuiCol_TextDisabled);

        bool map = tiles && tiles->isActive();
        if (map) {
            // tiles lightened (light theme) or darkened (dark theme) a little, so that the points stand out
            float a = 0.75f;
            tiles->draw(dl, p0, p1, cLat, mpp,
                        [&](double la, double lo) { double x, y; project(la, lo, x, y); return ImVec2((float)(cx + x), (float)(cy + y)); },
                        [&](double sx, double sy, double& la, double& lo) { unproject(sx - cx, sy - cy, la, lo); }, a);
        }
        if (haveData) {
            if (!map) { drawGrid(dl, p0, p1, cx, cy); }
            if (siteValid) { drawSite(dl, p0, p1, cx, cy, true); }
            drawPoints(dl, p0, p1, cx, cy, pts, hovered);
            if (siteValid) { drawSite(dl, p0, p1, cx, cy, false); }
            if (curValid) { drawCurrent(dl, cx, cy, curLat, curLon, curCourse); }
            drawScaleBar(dl, p0, p1);
            if (map) { drawAttribution(dl, p0, p1); }
            drawNorth(dl, p0, p1);
        }
        else {
            const char* t = ui_text::T("No GPS samples yet - connect the GPS receiver (softkey GPS...).");
            ImVec2 ts = ImGui::CalcTextSize(t);
            dl->AddText(ImVec2((float)cx - ts.x * 0.5f, (float)cy - ts.y * 0.5f), dimCol, t);
        }
        if (!pts.empty()) { lastDbm = pts.back().dbm; }
        drawLegend(dl, p0, p1);
        if (picking) {
            const char* t = ui_text::T("Click on the map to set the cell site (right click: cancel)");
            ImVec2 ts = ImGui::CalcTextSize(t);
            ImVec2 a((float)cx - ts.x * 0.5f - 8 * us, p0.y + 8 * us);
            dl->AddRectFilled(a, ImVec2(a.x + ts.x + 16 * us, a.y + ts.y + 8 * us), ImGui::GetColorU32(ImGuiCol_PopupBg), 4 * us);
            dl->AddText(ImVec2(a.x + 8 * us, a.y + 4 * us), textCol, t);
        }
        dl->AddRect(p0, p1, ImGui::GetColorU32(ImGuiCol_Border));
        dl->PopClipRect();
        if (picking && hovered) { ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); }

        if (hovered && hoverIdx >= 0 && hoverIdx < (int)pts.size() && !ImGui::IsItemActive()) { tooltip(pts[hoverIdx]); }
    }

private:
    double cLat = 0, cLon = 0, mpp = 10;
    bool viewSet = false, fitPending = false, dragged = false;
    double lastW = 800, lastH = 600;
    int hoverIdx = -1;
    std::vector<int> drawn;              // indices drawn this frame (oldest first)
    std::vector<unsigned char> occ;      // occupancy of 3x3 px cells

    void fit(const std::vector<DrivePoint>& pts, bool curValid, double curLat, double curLon, ImVec2 size) {
        double la0 = 1e9, la1 = -1e9, lo0 = 1e9, lo1 = -1e9;
        auto add = [&](double la, double lo) { la0 = (std::min)(la0, la); la1 = (std::max)(la1, la); lo0 = (std::min)(lo0, lo); lo1 = (std::max)(lo1, lo); };
        for (auto& p : pts) { add(p.lat, p.lon); }
        if (curValid) { add(curLat, curLon); }
        if (pts.empty() && !curValid && siteValid) { add(siteLat, siteLon); }
        if (la0 > la1) { return; }
        cLat = (la0 + la1) * 0.5;
        cLon = (lo0 + lo1) * 0.5;
        double ex = (lo1 - lo0) * drivemap::M_PER_DEG * std::cos(cLat * drivemap::PI / 180);
        double ey = (la1 - la0) * drivemap::M_PER_DEG;
        double margin = 40.0 * style::uiScale;
        double w = (std::max)(50.0, (double)size.x - 2 * margin), h = (std::max)(50.0, (double)size.y - 2 * margin);
        mpp = std::clamp((std::max)(ex / w, ey / h), 0.5, 20000.0);
        if (ex < 1 && ey < 1) { mpp = 2.0; }   // a single point
    }

    // grid in steps of 1-2-5 metres, fixed to the ground (does not move when panning)
    void drawGrid(ImDrawList* dl, ImVec2 p0, ImVec2 p1, double cx, double cy) {
        double step = drivemap::niceStep(mpp * 120.0 * style::uiScale);
        double kx = drivemap::M_PER_DEG * std::cos(cLat * drivemap::PI / 180), ky = drivemap::M_PER_DEG;
        ImVec4 bc = ImGui::GetStyleColorVec4(ImGuiCol_Border);
        bc.w *= 0.35f;
        ImU32 col = ImGui::ColorConvertFloat4ToU32(bc);
        double x0 = cLon * kx - (cx - p0.x) * mpp, x1 = cLon * kx + (p1.x - cx) * mpp;
        for (double g = std::ceil(x0 / step) * step; g <= x1; g += step) {
            float x = (float)(cx + (g - cLon * kx) / mpp);
            dl->AddLine(ImVec2(x, p0.y), ImVec2(x, p1.y), col);
        }
        double y0 = cLat * ky - (p1.y - cy) * mpp, y1 = cLat * ky + (cy - p0.y) * mpp;
        for (double g = std::ceil(y0 / step) * step; g <= y1; g += step) {
            float y = (float)(cy - (g - cLat * ky) / mpp);
            dl->AddLine(ImVec2(p0.x, y), ImVec2(p1.x, y), col);
        }
    }

    // rings: distance rings (under the track), otherwise the mast symbol and its label (on top)
    void drawSite(ImDrawList* dl, ImVec2 p0, ImVec2 p1, double cx, double cy, bool rings) {
        float us = style::uiScale;
        double x, y;
        project(siteLat, siteLon, x, y);
        ImVec2 s((float)(cx + x), (float)(cy + y));
        ImU32 col = ImGui::ColorConvertFloat4ToU32(tc::val());
        // distance rings up to the farthest corner of the view
        if (rings) {
            double far = 0;
            ImVec2 corners[4] = { p0, ImVec2(p1.x, p0.y), ImVec2(p0.x, p1.y), p1 };
            for (auto& c : corners) { far = (std::max)(far, std::hypot(c.x - s.x, c.y - s.y) * mpp); }
            double viewR = (std::min)(p1.x - p0.x, p1.y - p0.y) * 0.5 * mpp;
            double step = drivemap::niceStep(viewR / 2.5);
            ImVec4 rc = tc::val();
            rc.w = 0.55f;
            ImU32 ringCol = ImGui::ColorConvertFloat4ToU32(rc);
            int n = 0;
            for (double r = step; r <= far && n < 40; r += step, n++) {
                float rp = (float)(r / mpp);
                dl->AddCircle(s, rp, ringCol, 128, 1.0f * us);
                std::string t = drivemap::distText(r);
                dl->AddText(ImVec2(s.x + 3 * us, s.y - rp - ImGui::GetTextLineHeight()), ringCol, t.c_str());
            }
            return;
        }
        // mast symbol
        float h = 14 * us;
        dl->AddTriangleFilled(ImVec2(s.x, s.y - h), ImVec2(s.x - h * 0.45f, s.y + h * 0.35f), ImVec2(s.x + h * 0.45f, s.y + h * 0.35f), ImGui::GetColorU32(ImGuiCol_FrameBg));
        dl->AddTriangle(ImVec2(s.x, s.y - h), ImVec2(s.x - h * 0.45f, s.y + h * 0.35f), ImVec2(s.x + h * 0.45f, s.y + h * 0.35f), col, 2.0f * us);
        dl->AddCircleFilled(ImVec2(s.x, s.y - h), 3.0f * us, col);
        const char* lbl = ui_text::T("Cell site");
        ImVec2 ts = ImGui::CalcTextSize(lbl), la(s.x + h * 0.7f, s.y - h * 0.3f);
        ImVec4 bg = ImGui::GetStyleColorVec4(ImGuiCol_PopupBg);
        bg.w = 0.8f;
        dl->AddRectFilled(ImVec2(la.x - 2 * us, la.y - 1 * us), ImVec2(la.x + ts.x + 2 * us, la.y + ts.y + 1 * us), ImGui::ColorConvertFloat4ToU32(bg), 2 * us);
        dl->AddText(la, col, lbl);
    }

    ImU32 pointColor(const DrivePoint& p, bool& draw) const {
        draw = true;
        if (!p.service) { return IM_COL32(0, 0, 0, 255); }
        double v;
        switch (metric) {
        case cov::LEVEL: v = p.level; break;
        case cov::MER: v = p.mer; break;
        case cov::BER: v = (std::isnan(p.ber) || p.ber < 0) ? NAN : p.ber; break;
        default: v = std::isnan(p.crcPct) ? NAN : cov::quality(p.crcPct, crcLimit); break;
        }
        if (std::isnan(v)) { draw = false; return 0; }
        cov::Scale s = cov::scaleFor(metric, p.dbm, crcLimit);
        int r, g, b;
        cov::ramp(s.norm(v), r, g, b);
        return IM_COL32(r, g, b, 255);
    }

    // newest points first into a 3 px occupancy grid, so that zoomed out
    // only one point per cell is drawn; then drawn oldest first
    void drawPoints(ImDrawList* dl, ImVec2 p0, ImVec2 p1, double cx, double cy, const std::vector<DrivePoint>& pts, bool hovered) {
        float us = style::uiScale;
        const int CELL = 3;
        int gw = (int)((p1.x - p0.x) / CELL) + 1, gh = (int)((p1.y - p0.y) / CELL) + 1;
        occ.assign((size_t)gw * gh, 0);
        drawn.clear();
        hoverIdx = -1;
        float best = 8.0f * us;
        ImVec2 m = ImGui::GetIO().MousePos;
        for (int i = (int)pts.size() - 1; i >= 0; i--) {
            double x, y;
            project(pts[i].lat, pts[i].lon, x, y);
            double sx = cx + x, sy = cy + y;
            if (sx < p0.x - 5 || sy < p0.y - 5 || sx > p1.x + 5 || sy > p1.y + 5) { continue; }
            int gx = std::clamp((int)((sx - p0.x) / CELL), 0, gw - 1), gy = std::clamp((int)((sy - p0.y) / CELL), 0, gh - 1);
            unsigned char& o = occ[(size_t)gy * gw + gx];
            if (o) { continue; }
            o = 1;
            drawn.push_back(i);
            if (hovered) {
                float d = (float)std::hypot(sx - m.x, sy - m.y);
                if (d < best) { best = d; hoverIdx = i; }
            }
        }
        float r = 3.5f * us;
        ImU32 rim = IM_COL32(200, 200, 200, 255);
        for (auto it = drawn.rbegin(); it != drawn.rend(); ++it) {
            const DrivePoint& p = pts[*it];
            bool ok;
            ImU32 c = pointColor(p, ok);
            if (!ok) { continue; }
            double x, y;
            project(p.lat, p.lon, x, y);
            ImVec2 s((float)(cx + x), (float)(cy + y));
            dl->AddCircleFilled(s, r, c, 10);
            // thin rim so that the colours stand out on a map: dark, light for "no service"
            dl->AddCircle(s, r, p.service ? IM_COL32(20, 20, 20, 200) : rim, 10, 1.0f);
        }
        if (hoverIdx >= 0) {
            double x, y;
            project(pts[hoverIdx].lat, pts[hoverIdx].lon, x, y);
            dl->AddCircle(ImVec2((float)(cx + x), (float)(cy + y)), r + 3 * us, ImGui::GetColorU32(ImGuiCol_Text), 16, 1.5f * us);
        }
    }

    // white arrow in the direction of travel (circle when standing)
    void drawCurrent(ImDrawList* dl, double cx, double cy, double lat, double lon, float course) {
        float us = style::uiScale;
        double x, y;
        project(lat, lon, x, y);
        ImVec2 s((float)(cx + x), (float)(cy + y));
        ImU32 fill = IM_COL32(255, 255, 255, 255), edge = IM_COL32(20, 20, 20, 255);
        if (std::isnan(course)) {
            dl->AddCircleFilled(s, 7 * us, fill, 20);
            dl->AddCircle(s, 7 * us, edge, 20, 2 * us);
            return;
        }
        double a = course * drivemap::PI / 180;
        auto pt = [&](double fwd, double side) { return ImVec2((float)(s.x + fwd * std::sin(a) + side * std::cos(a)), (float)(s.y - fwd * std::cos(a) + side * std::sin(a))); };
        float L = 12 * us;
        ImVec2 tip = pt(L, 0), l = pt(-L * 0.7, -L * 0.7), r = pt(-L * 0.7, L * 0.7), back = pt(-L * 0.3, 0);
        dl->AddTriangleFilled(tip, l, back, fill);
        dl->AddTriangleFilled(tip, back, r, fill);
        ImVec2 poly[4] = { tip, r, back, l };
        dl->AddPolyline(poly, 4, edge, ImDrawFlags_Closed, 2 * us);
    }

    void drawScaleBar(ImDrawList* dl, ImVec2 p0, ImVec2 p1) {
        (void)p0;
        float us = style::uiScale;
        double len = drivemap::niceStep(mpp * 140.0 * us);
        float px = (float)(len / mpp);
        float x1 = p1.x - 14 * us, x0 = x1 - px, y = p1.y - 14 * us;
        ImU32 c = ImGui::GetColorU32(ImGuiCol_Text);
        backdrop(dl, ImVec2(x0 - 6 * us, y - 8 * us - ImGui::GetTextLineHeight()), ImVec2(x1 + 6 * us, y + 5 * us));
        dl->AddLine(ImVec2(x0, y), ImVec2(x1, y), c, 2 * us);
        dl->AddLine(ImVec2(x0, y - 6 * us), ImVec2(x0, y), c, 2 * us);
        dl->AddLine(ImVec2(x1, y - 6 * us), ImVec2(x1, y), c, 2 * us);
        std::string t = drivemap::distText(len);
        ImVec2 ts = ImGui::CalcTextSize(t.c_str());
        dl->AddText(ImVec2((x0 + x1) * 0.5f - ts.x * 0.5f, y - 6 * us - ts.y), c, t.c_str());
    }

    static void backdrop(ImDrawList* dl, ImVec2 a, ImVec2 b) {
        ImVec4 bg = ImGui::GetStyleColorVec4(ImGuiCol_PopupBg);
        bg.w = 0.7f;
        dl->AddRectFilled(a, b, ImGui::ColorConvertFloat4ToU32(bg), 3 * style::uiScale);
    }

    // source of the map, bottom right above the scale bar (required by the providers)
    void drawAttribution(ImDrawList* dl, ImVec2 p0, ImVec2 p1) {
        if (attribution.empty()) { return; }
        float us = style::uiScale;
        ImVec2 ts = ImGui::CalcTextSize(attribution.c_str(), nullptr, false, (p1.x - p0.x) * 0.6f);
        ImVec2 a(p1.x - ts.x - 14 * us, p1.y - 14 * us - ImGui::GetTextLineHeight() - 10 * us - ts.y - 4 * us);
        ImVec4 bg = ImGui::GetStyleColorVec4(ImGuiCol_PopupBg);
        bg.w = 0.8f;
        dl->AddRectFilled(ImVec2(a.x - 4 * us, a.y - 2 * us), ImVec2(a.x + ts.x + 4 * us, a.y + ts.y + 2 * us), ImGui::ColorConvertFloat4ToU32(bg), 2 * us);
        dl->AddText(ImGui::GetFont(), ImGui::GetFontSize(), a, ImGui::GetColorU32(ImGuiCol_Text), attribution.c_str(), nullptr, (p1.x - p0.x) * 0.6f);
    }

    void drawNorth(ImDrawList* dl, ImVec2 p0, ImVec2 p1) {
        float us = style::uiScale;
        ImU32 c = ImGui::GetColorU32(ImGuiCol_Text);
        ImVec2 ts = ImGui::CalcTextSize("N");
        float x = p1.x - 22 * us, y = p0.y + 10 * us;
        backdrop(dl, ImVec2(x - 11 * us, y - 4 * us), ImVec2(x + 11 * us, y + 22 * us + ts.y));
        dl->AddTriangleFilled(ImVec2(x, y), ImVec2(x - 7 * us, y + 18 * us), ImVec2(x + 7 * us, y + 18 * us), c);
        dl->AddText(ImVec2(x - ts.x * 0.5f, y + 20 * us), c, "N");
    }

    // like the legend of the KMZ: title, colour bar with the end values, "no service"
    void drawLegend(ImDrawList* dl, ImVec2 p0, ImVec2 p1) {
        (void)p1;
        float us = style::uiScale;
        cov::Scale s = cov::scaleFor(metric, lastDbm, crcLimit);
        float lh = ImGui::GetTextLineHeight(), pad = 8 * us, bw = 200 * us, bh = 12 * us;
        float h = pad + lh + 4 * us + bh + 4 * us + lh + 4 * us + lh + pad;
        ImVec2 a(p0.x + 10 * us, p1.y - 10 * us - h);
        ImVec4 bg = ImGui::GetStyleColorVec4(ImGuiCol_PopupBg);
        bg.w = 0.85f;
        dl->AddRectFilled(a, ImVec2(a.x + bw + 2 * pad, a.y + h), ImGui::ColorConvertFloat4ToU32(bg), 4 * us);
        ImU32 tcol = ImGui::GetColorU32(ImGuiCol_Text);
        float y = a.y + pad;
        dl->AddText(ImVec2(a.x + pad, y), tcol, ui_text::T(s.title.c_str()));
        y += lh + 4 * us;
        int N = (int)bw;
        for (int i = 0; i < N; i++) {
            double t = (double)i / (N - 1);
            if (s.steps) { t = t < 1.0 / 3 ? 0.0 : t < 2.0 / 3 ? 0.5 : 1.0; }
            int r, g, b;
            cov::ramp(t, r, g, b);
            dl->AddRectFilled(ImVec2(a.x + pad + i, y), ImVec2(a.x + pad + i + 1, y + bh), IM_COL32(r, g, b, 255));
        }
        y += bh + 4 * us;
        dl->AddText(ImVec2(a.x + pad, y), tcol, s.loText.c_str());
        ImVec2 hs = ImGui::CalcTextSize(s.hiText.c_str());
        dl->AddText(ImVec2(a.x + pad + bw - hs.x, y), tcol, s.hiText.c_str());
        y += lh + 4 * us;
        ImVec2 dc(a.x + pad + 6 * us, y + lh * 0.5f);
        dl->AddCircleFilled(dc, 5 * us, IM_COL32(0, 0, 0, 255), 12);
        dl->AddCircle(dc, 5 * us, IM_COL32(200, 200, 200, 255), 12, 1.0f);
        dl->AddText(ImVec2(a.x + pad + 16 * us, y), tcol, ui_text::T("no service"));
    }

    static void tipRow(const char* k, const char* fmt, ...) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        tw::TextDisabled("%s", ui_text::T(k));
        ImGui::TableSetColumnIndex(1);
        char b[96];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(b, sizeof(b), fmt, ap);
        va_end(ap);
        ImGui::TextUnformatted(b);
    }

    void tooltip(const DrivePoint& p) {
        ImGui::BeginTooltip();
        char tb[32] = "";
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &p.t);
#else
        localtime_r(&p.t, &lt);
#endif
        strftime(tb, sizeof(tb), "%Y-%m-%d %H:%M:%S", &lt);
        if (ImGui::BeginTable("##drive_tip", 2)) {
            auto& row = tipRow;
            row("Time", "%s", tb);
            row("Level", "%.1f %s", p.level, p.dbm ? "dBm" : "dBFS");
            row("MER", "%.1f dB", p.mer);
            if (!std::isnan(p.ber) && p.ber >= 0) { row("BER", "%.2e", p.ber); }
            row("CRC errors", "%.1f %%", p.crcPct);
            if (p.mcc != 0) { row("Cell", "MCC %d MNC %d CC %d", p.mcc, p.mnc, p.cc); }
            if (p.dlHz > 0) { row("Downlink", "%.4f MHz", p.dlHz / 1e6); }
            row("Service", "%s", ui_text::T(p.service ? "yes" : "no"));
            if (siteValid) { row("Distance to cell site", "%s", drivemap::distText(std::round(drivemap::distanceM(siteLat, siteLon, p.lat, p.lon))).c_str()); }
            ImGui::EndTable();
        }
        ImGui::EndTooltip();
    }

public:
    bool lastDbm = false;         // unit of the level scale (from the newest point)
};
