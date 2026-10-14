#pragma once
// Constellation display for pi/4-DQPSK with the ETSI EN 300 392-2 modulation
// accuracy limits drawn as target zones around the ideal symbol positions:
//   inner zone: vector error magnitude 10 % (RMS limit)
//   outer ring: vector error magnitude 30 % (peak limit)
// Points are yellow, and turn green when the measured EVM is within the
// RMS limit.

#include <cmath>
#include <cstring>
#include <mutex>
#include <imgui.h>
#include <dsp/types.h>

class TetraConstellation {
public:
    static const int N = 1024;

    // Called from the DSP thread
    void update(const dsp::complex_t* data, int count) {
        std::lock_guard<std::mutex> lck(mtx);
        int n = count < N ? count : N;
        memcpy(buf, data, n * sizeof(dsp::complex_t));
        filled = n;
    }

    // eightPoints: ideal points at k*45 deg, otherwise at 45+k*90 deg
    // evmRmsPct < 0: unknown (no sync)
    void draw(const char* id, float size, bool eightPoints, float evmRmsPct) {
        dsp::complex_t local[N];
        int n;
        {
            std::lock_guard<std::mutex> lck(mtx);
            n = filled;
            memcpy(local, buf, n * sizeof(dsp::complex_t));
        }

        ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImVec2 p1 = ImVec2(p0.x + size, p0.y + size);
        ImGui::InvisibleButton(id, ImVec2(size, size));
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 c = ImVec2((p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f);
        float R = size * 0.36f; // radius of the ideal points (unit circle)

        dl->AddRectFilled(p0, p1, IM_COL32(8, 8, 12, 255));
        dl->AddLine(ImVec2(p0.x, c.y), ImVec2(p1.x, c.y), IM_COL32(60, 60, 70, 255));
        dl->AddLine(ImVec2(c.x, p0.y), ImVec2(c.x, p1.y), IM_COL32(60, 60, 70, 255));
        dl->AddCircle(c, R, IM_COL32(45, 45, 55, 255), 96);

        // ETSI target zones
        int pts = eightPoints ? 8 : 4;
        for (int k = 0; k < pts; k++) {
            float a = eightPoints ? (float)k * 3.14159265f / 4.0f : 3.14159265f / 4.0f + (float)k * 3.14159265f / 2.0f;
            ImVec2 ip = ImVec2(c.x + R * cosf(a), c.y - R * sinf(a));
            dl->AddCircleFilled(ip, R * 0.30f, IM_COL32(255, 255, 255, 14), 32);
            dl->AddCircle(ip, R * 0.30f, IM_COL32(255, 255, 255, 40), 32);
            dl->AddCircleFilled(ip, R * 0.10f, IM_COL32(120, 255, 120, 45), 24);
            dl->AddCircle(ip, R * 0.10f, IM_COL32(120, 255, 120, 110), 24);
        }

        // Normalize by the mean magnitude so the ideal points sit on the circle
        double mag = 0.0;
        for (int i = 0; i < n; i++) { mag += sqrt((double)local[i].re * local[i].re + (double)local[i].im * local[i].im); }
        float scale = (n > 0 && mag > 1e-9) ? (float)(R / (mag / n)) : R;

        bool good = evmRmsPct >= 0.0f && evmRmsPct <= 10.0f;
        ImU32 col = good ? IM_COL32(60, 255, 60, 230) : IM_COL32(255, 230, 40, 230);
        dl->PushClipRect(p0, p1, true);
        for (int i = 0; i < n; i++) {
            float x = c.x + local[i].re * scale;
            float y = c.y - local[i].im * scale;
            dl->AddRectFilled(ImVec2(x - 1.0f, y - 1.0f), ImVec2(x + 1.0f, y + 1.0f), col);
        }
        dl->PopClipRect();

        // Legend
        char txt[96];
        if (evmRmsPct >= 0.0f) {
            snprintf(txt, sizeof(txt), "EVM %.1f %% %s", evmRmsPct, good ? "(ETSI ok)" : "(> 10 %)");
        }
        else {
            snprintf(txt, sizeof(txt), "no sync");
        }
        dl->AddText(ImVec2(p0.x + 6, p0.y + 4), good ? IM_COL32(60, 255, 60, 255) : IM_COL32(255, 230, 40, 255), txt);
        dl->AddText(ImVec2(p0.x + 6, p1.y - ImGui::GetFontSize() - 4), IM_COL32(150, 150, 160, 255), "zones: 10 % RMS / 30 % peak");
    }

private:
    std::mutex mtx;
    dsp::complex_t buf[N];
    int filled = 0;
};
