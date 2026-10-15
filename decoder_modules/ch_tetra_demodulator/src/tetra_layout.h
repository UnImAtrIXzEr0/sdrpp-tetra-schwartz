#pragma once
// Width-aware layout helpers: an element that would not fit on the current
// line any more goes to the next line instead of disappearing to the right.
#include <imgui.h>
#include <cstdarg>
#include <cstdio>
#include <algorithm>
#include <cstring>

namespace tw {
    // right edge of the usable content area of the current window
    inline float rightEdge() {
        return ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x;
    }

    // "Same line as the label, if the following value fits": set by SameNext(),
    // consumed by the next tw::Text* call (which knows its width)
    inline bool& pendingSame() { static bool p = false; return p; }
    inline void SameNext() { pendingSame() = true; }

    // Continue on the line of the last item if 'w' more pixels fit, else start
    // a new line (indented a little so it reads as a continuation).
    inline bool SameLineFor(float w) {
        pendingSame() = false;
        float x = ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x;
        if (x + w <= rightEdge()) {
            ImGui::SameLine();
            return true;
        }
        // new line, slightly indented as a continuation (if that still fits)
        if (ImGui::GetCursorScreenPos().x + ImGui::GetFontSize() + w <= rightEdge()) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetFontSize());
        }
        return false;
    }

    inline float textWidth(const char* s) { return ImGui::CalcTextSize(s, nullptr, true).x; }
    inline float checkboxWidth(const char* label) {
        return ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + textWidth(label);
    }
    inline float smallButtonWidth(const char* label) { return textWidth(label) + 2.0f * ImGui::GetStyle().FramePadding.x; }
    inline float buttonWidth(const char* label) { return textWidth(label) + 2.0f * ImGui::GetStyle().FramePadding.x; }

    inline void vfmt(char* buf, size_t n, const char* fmt, va_list ap) { vsnprintf(buf, n, fmt, ap); }

    // Table cells: single line, clipped with "..." (full text as tooltip)
    // instead of wrapping. Active inside a CellScope.
    inline bool& cellMode() { static bool c = false; return c; }
    struct CellScope {
        bool prev;
        CellScope() : prev(cellMode()) { cellMode() = true; }
        ~CellScope() { cellMode() = prev; }
    };

    // Fit 'buf' into the remaining width: in cell mode truncate (returns true
    // and keeps the full text in 'full'), otherwise wrap if needed.
    struct FitText {
        bool wrap = false, cut = false;
        char full[512];
        FitText(char* buf, size_t n) {
            float avail = ImGui::GetContentRegionAvail().x;
            if (textWidth(buf) <= avail || avail <= 0) { return; }
            if (!cellMode()) { ImGui::PushTextWrapPos(0.0f); wrap = true; return; }
            snprintf(full, sizeof(full), "%s", buf);
            size_t total = strlen(buf);
            float dots = textWidth("...");
            // binary search for the longest prefix that fits (cells are drawn
            // every frame, a linear search would be slow for long texts)
            auto prefixW = [&](size_t l) { return ImGui::CalcTextSize(buf, buf + l).x; };
            size_t lo = 0, hi = total;
            while (lo < hi) {
                size_t mid = (lo + hi + 1) / 2;
                if (prefixW(mid) + dots <= avail) { lo = mid; } else { hi = mid - 1; }
            }
            size_t len = lo;
            // do not cut inside a UTF-8 sequence
            while (len > 0 && (((unsigned char)buf[len]) & 0xC0) == 0x80) { len--; }
            if (len + 4 < n) { buf[len] = 0; strcat(buf, "..."); }
            cut = true;
        }
        void done() {
            if (wrap) { ImGui::PopTextWrapPos(); wrap = false; }
            if (cut && ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", full); }
        }
    };

    inline void flushPending(float w) {
        if (pendingSame()) { pendingSame() = false; SameLineFor(w); }
    }
    // drop-in replacements for ImGui::Text*: behave identically unless a
    // SameNext() is pending; too long texts wrap (cells: clip with "...")
    inline void TextColored(const ImVec4& col, const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        flushPending(textWidth(buf));
        FitText f(buf, sizeof(buf));
        ImGui::TextColored(col, "%s", buf);
        f.done();
    }
    inline void Text(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        flushPending(textWidth(buf));
        FitText f(buf, sizeof(buf));
        ImGui::TextUnformatted(buf);
        f.done();
    }
    inline void TextU(const char* text) { Text("%s", text); }
    inline void TextDisabled(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        flushPending(textWidth(buf));
        FitText f(buf, sizeof(buf));
        ImGui::TextDisabled("%s", buf);
        f.done();
    }

    inline void SameTextColored(const ImVec4& col, const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        SameLineFor(textWidth(buf));
        ImGui::TextColored(col, "%s", buf);
    }
    inline void SameText(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        SameLineFor(textWidth(buf));
        ImGui::TextUnformatted(buf);
    }
    inline void SameTextDisabled(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        SameLineFor(textWidth(buf));
        ImGui::TextDisabled("%s", buf);
    }
    inline bool SameCheckbox(const char* label, bool* v) {
        SameLineFor(checkboxWidth(label));
        return ImGui::Checkbox(label, v);
    }
    inline bool SameSmallButton(const char* label) {
        SameLineFor(smallButtonWidth(label));
        return ImGui::SmallButton(label);
    }
    inline bool SameButton(const char* label, const ImVec2& size = ImVec2(0, 0)) {
        SameLineFor(size.x > 0 ? size.x : buttonWidth(label));
        return ImGui::Button(label, size);
    }
    // Button next to the last item if it fits, otherwise a full-width button
    // on its own line (e.g. "Calibrate SDR" below its measurement)
    inline bool SameButtonOrFull(const char* label) {
        float x = ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x;
        if (x + smallButtonWidth(label) <= rightEdge()) {
            ImGui::SameLine();
            return ImGui::SmallButton(label);
        }
        return ImGui::Button(label, ImVec2(ImGui::GetContentRegionAvail().x, 0));
    }

    // A row of checkboxes: one line if everything fits, otherwise the label on
    // its own line and the boxes in as many columns as fit (2 typically).
    struct CheckRow {
        const char* labels[8];
        bool* values[8];
        bool enabled[8];
        int n = 0;
        void add(const char* l, bool* v, bool en = true) { if (n < 8) { labels[n] = l; values[n] = v; enabled[n] = en; n++; } }
        // returns the index of the box that changed, or -1
        int draw(const char* title) {
            const ImGuiStyle& st = ImGui::GetStyle();
            float avail = ImGui::GetContentRegionAvail().x;
            float total = textWidth(title) + st.ItemSpacing.x;
            float maxw = 0;
            for (int i = 0; i < n; i++) {
                float w = checkboxWidth(labels[i]);
                total += w + st.ItemSpacing.x;
                maxw = (std::max)(maxw, w);
            }
            int changed = -1;
            auto box = [&](int i) {
                if (!enabled[i]) { ImGui::BeginDisabled(); }
                if (ImGui::Checkbox(labels[i], values[i])) { changed = i; }
                if (!enabled[i]) { ImGui::EndDisabled(); }
            };
            if (total <= avail) {
                ImGui::TextUnformatted(title);
                for (int i = 0; i < n; i++) { ImGui::SameLine(); box(i); }
                return changed;
            }
            ImGui::TextUnformatted(title);
            int cols = (std::max)(1, (std::min)(n, (int)((avail + st.ItemSpacing.x) / (maxw + st.ItemSpacing.x))));
            float colW = avail / cols;
            float x0 = ImGui::GetCursorPosX();
            for (int i = 0; i < n; i++) {
                int c = i % cols;
                if (c > 0) { ImGui::SameLine(x0 + c * colW); }
                box(i);
            }
            return changed;
        }
    };
}
