#include <version.h>
#include <gui/dialogs/loading_screen.h>
#include <gui/main_window.h>
#include <imgui.h>
#include <gui/icons.h>
#include <gui/style.h>
#include <credits.h>
#include <gui/gui.h>
#include <backend.h>
#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cmath>

// Start-up screen in the style of a measuring instrument booting: fixed light
// grey / blue colours, independent of the selected theme (the theme is not
// applied yet while loading, which made the old screen unreadable).
namespace LoadingScreen {
    ImVec2 imageSize(128.0f, 128.0f);

    void init() {
        imageSize = ImVec2(128.0f * style::uiScale, 128.0f * style::uiScale);
    }

    static const ImU32 COL_BG = IM_COL32(233, 236, 239, 255);
    static const ImU32 COL_BAND = IM_COL32(0, 58, 108, 255);
    static const ImU32 COL_ACCENT = IM_COL32(86, 160, 220, 255);
    static const ImU32 COL_BLUE = IM_COL32(0, 58, 108, 255);
    static const ImU32 COL_GREY = IM_COL32(96, 106, 116, 255);
    static const ImU32 COL_DARK = IM_COL32(10, 21, 32, 255);
    static const ImU32 COL_TRACK = IM_COL32(200, 207, 214, 255);
    static const ImU32 COL_WHITE = IM_COL32(255, 255, 255, 255);

    static void text(ImDrawList* dl, ImFont* f, ImVec2 pos, ImU32 col, const char* s) {
        if (!f) { f = ImGui::GetFont(); }
        dl->AddText(f, f->FontSize, pos, col, s);
    }

    static ImVec2 textSize(ImFont* f, const char* s) {
        if (!f) { f = ImGui::GetFont(); }
        return f->CalcTextSizeA(f->FontSize, FLT_MAX, 0.0f, s);
    }

    void show(std::string msg) {
        backend::beginFrame();

        const float s = style::uiScale;
        ImGuiViewport* vp = ImGui::GetMainViewport();
        ImVec2 p0 = vp->Pos;
        ImVec2 sz = vp->Size;
        ImVec2 p1 = ImVec2(p0.x + sz.x, p0.y + sz.y);
        ImDrawList* dl = ImGui::GetBackgroundDrawList();

        // Background, top band with edition name, thin accent line
        dl->AddRectFilled(p0, p1, COL_BG);
        float bandH = 44.0f * s;
        dl->AddRectFilled(p0, ImVec2(p1.x, p0.y + bandH), COL_BAND);
        dl->AddRectFilled(ImVec2(p0.x, p0.y + bandH), ImVec2(p1.x, p0.y + bandH + 3.0f * s), COL_ACCENT);
        {
            ImFont* f = style::baseFont;
            float y = p0.y + (bandH - (f ? f->FontSize : 16.0f)) * 0.5f;
            text(dl, f, ImVec2(p0.x + 16.0f * s, y), COL_WHITE, EDITION_NAME);
            const char* r = "experimental beta";
            text(dl, f, ImVec2(p1.x - textSize(f, r).x - 16.0f * s, y), COL_WHITE, r);
        }

        // Centre block: logo, name, version
        const char* l1 = "SDR++";
        const char* l2 = "TETRA-SCHWARTZ";
        ImFont* big = style::bigFont;
        ImVec2 s1 = textSize(big, l1);
        ImVec2 s2 = textSize(big, l2);
        float textW = (std::max)(s1.x, s2.x);
        float gap = 28.0f * s;
        bool haveLogo = icons::LOGO != (ImTextureID)0;
        float blockW = (haveLogo ? imageSize.x + gap : 0.0f) + textW;
        float blockH = (std::max)(imageSize.y, s1.y + s2.y);
        ImVec2 c0 = ImVec2(p0.x + (sz.x - blockW) * 0.5f, p0.y + (sz.y - blockH) * 0.5f - 30.0f * s);
        float tx = c0.x;
        if (haveLogo) {
            dl->AddImage(icons::LOGO, c0, ImVec2(c0.x + imageSize.x, c0.y + imageSize.y));
            tx += imageSize.x + gap;
        }
        float ty = c0.y + (blockH - (s1.y + s2.y)) * 0.5f;
        text(dl, big, ImVec2(tx, ty), COL_BLUE, l1);
        text(dl, big, ImVec2(tx, ty + s1.y), COL_GREY, l2);

        float y = c0.y + blockH + 26.0f * s;
        std::string ver = std::string("Version ") + EDITION_VERSION + "   |   based on SDR++ by Alexandre Rouma (ON5RYZ)";
        ImVec2 vs = textSize(style::baseFont, ver.c_str());
        text(dl, style::baseFont, ImVec2(p0.x + (sz.x - vs.x) * 0.5f, y), COL_DARK, ver.c_str());

        // Status line and an indeterminate progress bar
        y += vs.y + 28.0f * s;
        float barW = (std::min)(520.0f * s, sz.x * 0.6f);
        float barH = 4.0f * s;
        ImVec2 b0 = ImVec2(p0.x + (sz.x - barW) * 0.5f, y);
        dl->AddRectFilled(b0, ImVec2(b0.x + barW, b0.y + barH), COL_TRACK);
        double t = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        float segW = barW * 0.25f;
        float ph = (float)std::fmod(t * 0.6, 1.0);
        float sx = b0.x + (barW + segW) * ph - segW;
        float sx0 = (std::max)(sx, b0.x), sx1 = (std::min)(sx + segW, b0.x + barW);
        if (sx1 > sx0) { dl->AddRectFilled(ImVec2(sx0, b0.y), ImVec2(sx1, b0.y + barH), COL_ACCENT); }
        ImVec2 ms = textSize(style::baseFont, msg.c_str());
        text(dl, style::baseFont, ImVec2(p0.x + (sz.x - ms.x) * 0.5f, b0.y + barH + 10.0f * s), COL_GREY, msg.c_str());

        // Footer
        const char* foot = "SDR++ by Alexandre Rouma (ON5RYZ)  -  SDR++Brown by sannysanoff  -  TETRA-SCHWARTZ by UnImAtrIXzEr0";
        ImFont* tf = style::tinyFont ? style::tinyFont : style::baseFont;
        ImVec2 fs = textSize(tf, foot);
        dl->AddRectFilled(ImVec2(p0.x, p1.y - fs.y - 20.0f * s), p1, IM_COL32(220, 225, 230, 255));
        text(dl, tf, ImVec2(p0.x + (sz.x - fs.x) * 0.5f, p1.y - fs.y - 10.0f * s), COL_GREY, foot);

        backend::render(false);
    }
}
