#pragma once
// TETRA constellation display with the ETSI vector error tolerance drawn
// around every ideal symbol position.
//  - EN 300 394-1 (BS transmitter): RMS vector error <= 10 %, peak <= 30 %
//  - dots are yellow, and turn green while the measured RMS EVM is within
//    the limit (with symbol sync)

#include <cmath>
#include <cstring>
#include <mutex>
#include <imgui.h>
#include <imgui_internal.h>
#include <dsp/types.h>

class TetraConstellationView {
public:
    // Colours (defaults: dark instrument display). A theme can override
    // them with its "Constellation*" keys, see drawConstellation() in main.cpp.
    struct Palette {
        ImU32 bg = IM_COL32(8, 10, 14, 255);
        ImU32 border = IM_COL32(60, 66, 76, 255);
        ImU32 grid = IM_COL32(48, 54, 64, 255);
        ImU32 peakZone = IM_COL32(120, 140, 170, 28);
        ImU32 rmsZone = IM_COL32(150, 200, 255, 45);
        ImU32 rmsRing = IM_COL32(150, 200, 255, 110);
        ImU32 marker = IM_COL32(200, 220, 255, 160);
        ImU32 dotGood = IM_COL32(40, 235, 90, 255);
        ImU32 dotBad = IM_COL32(255, 220, 0, 255);
        ImU32 text = IM_COL32(150, 170, 200, 255);
    };
    Palette pal;

    static const int N = 1024;
    static constexpr float RMS_LIMIT = 0.10f;
    static constexpr float PEAK_LIMIT = 0.30f;

    void set(const dsp::complex_t* data, int count) {
        if (count > N) { count = N; }
        std::lock_guard<std::mutex> lck(mtx);
        memcpy(buf, data, count * sizeof(dsp::complex_t));
        n = count;
    }

    // size.x = 0 -> available width; square by default
    void draw(const char* id, ImVec2 size, bool eightPoint, bool good, bool sync) {
        ImGuiWindow* window = ImGui::GetCurrentWindow();
        if (window->SkipItems) { return; }
        float avail = ImGui::GetContentRegionAvail().x;
        if (size.x <= 0) { size.x = avail; }
        if (size.y <= 0) { size.y = size.x; }
        ImVec2 p0 = window->DC.CursorPos;
        ImRect bb(p0, ImVec2(p0.x + size.x, p0.y + size.y));
        ImGui::ItemSize(size);
        if (!ImGui::ItemAdd(bb, ImGui::GetID(id))) { return; }

        ImDrawList* dl = window->DrawList;
        ImVec2 c((bb.Min.x + bb.Max.x) * 0.5f, (bb.Min.y + bb.Max.y) * 0.5f);
        float half = (std::fmin)(size.x, size.y) * 0.5f;
        float R = half * 0.72f; // radius of the ideal constellation points

        // Background and grid
        dl->AddRectFilled(bb.Min, bb.Max, pal.bg);
        dl->AddRect(bb.Min, bb.Max, pal.border);
        ImU32 grid = pal.grid;
        dl->AddLine(ImVec2(bb.Min.x, c.y), ImVec2(bb.Max.x, c.y), grid);
        dl->AddLine(ImVec2(c.x, bb.Min.y), ImVec2(c.x, bb.Max.y), grid);
        dl->AddCircle(c, R, grid, 96);

        // Ideal points with tolerance areas
        int nIdeal = eightPoint ? 8 : 4;
        for (int k = 0; k < nIdeal; k++) {
            float a = eightPoint ? (float)k * 3.14159265f / 4.0f : 3.14159265f / 4.0f + (float)k * 3.14159265f / 2.0f;
            ImVec2 ip(c.x + R * cosf(a), c.y - R * sinf(a));
            dl->AddCircleFilled(ip, R * PEAK_LIMIT, pal.peakZone, 48);  // peak limit, subtle
            dl->AddCircleFilled(ip, R * RMS_LIMIT, pal.rmsZone, 32);   // RMS limit
            dl->AddCircle(ip, R * RMS_LIMIT, pal.rmsRing, 32);
            dl->AddLine(ImVec2(ip.x - 3, ip.y), ImVec2(ip.x + 3, ip.y), pal.marker);
            dl->AddLine(ImVec2(ip.x, ip.y - 3), ImVec2(ip.x, ip.y + 3), pal.marker);
        }

        // Symbols, normalized to their mean magnitude
        ImU32 dot = (good && sync) ? pal.dotGood : pal.dotBad;
        dsp::complex_t local[N];
        int cnt;
        {
            std::lock_guard<std::mutex> lck(mtx);
            cnt = n;
            memcpy(local, buf, cnt * sizeof(dsp::complex_t));
        }
        double mag = 0.0;
        for (int i = 0; i < cnt; i++) { mag += sqrt((double)local[i].re * local[i].re + (double)local[i].im * local[i].im); }
        float scale = (cnt && mag > 1e-9) ? (float)(R / (mag / cnt)) : R;
        dl->PushClipRect(bb.Min, bb.Max, true);
        for (int i = 0; i < cnt; i++) {
            ImVec2 p(c.x + local[i].re * scale, c.y - local[i].im * scale);
            dl->AddRectFilled(ImVec2(p.x - 1.0f, p.y - 1.0f), ImVec2(p.x + 1.5f, p.y + 1.5f), dot);
        }
        dl->PopClipRect();

        // Legend
        const char* lg = (good && sync) ? "EVM within ETSI limit" : (sync ? "EVM above ETSI limit" : "no symbol sync");
        dl->AddText(ImVec2(bb.Min.x + 6, bb.Max.y - ImGui::GetFontSize() - 4),
                    (good && sync) ? pal.dotGood : pal.dotBad, lg);
        dl->AddText(ImVec2(bb.Min.x + 6, bb.Min.y + 4), pal.text, "circles: ETSI 10 % rms / 30 % peak");
    }

private:
    std::mutex mtx;
    dsp::complex_t buf[N];
    int n = 0;
};
