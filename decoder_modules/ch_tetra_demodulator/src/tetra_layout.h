#pragma once
// Layout helpers with an instrument-style value column (see valueSameLine):
// text never wraps or jumps with the values; controls go to the next line only
// if they do not fit (with hysteresis).
#include <imgui.h>
#include <cstdarg>
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <unordered_map>

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

    // ---- Instrument-style layout (v0.23.4) ----
    // Text lines never wrap: a "Label:" line puts its values into a value
    // column at a fixed x (the widest label of the window, at most 45 % of the
    // width), further values follow on the same line, and whatever does not fit
    // is cut with "..." (full text as tooltip). The layout depends only on the
    // window width and the (static) labels, never on the values - nothing moves
    // when a value changes. Only when the window is too narrow for a readable
    // value column the values go to a fixed line below their label.
    struct LabelCol { float maxW = 0.0f; };
    inline std::unordered_map<ImGuiID, LabelCol>& labelCols() { static std::unordered_map<ImGuiID, LabelCol> m; return m; }
    inline bool& labelPending() { static bool p = false; return p; }
    inline float& labelStartX() { static float x = 0.0f; return x; }
    inline float& labelEndX() { static float x = 0.0f; return x; }
    inline bool isLabel(const char* t) {
        size_t n = strlen(t);
        while (n > 0 && t[n - 1] == ' ') { n--; }
        return n > 1 && t[n - 1] == ':' && !strchr(t, '\n');
    }
    // after a label item: remember it (window-scoped key, labels are static)
    inline void noteLabel(const char* t) {
        labelPending() = false;
        if (!isLabel(t)) { return; }
        LabelCol& c = labelCols()[ImGui::GetID("##tw_labelcol")];
        float w = ImGui::GetItemRectSize().x;
        if (w > c.maxW) { c.maxW = w; }
        labelPending() = true;
        labelStartX() = ImGui::GetItemRectMin().x;
        labelEndX() = ImGui::GetItemRectMax().x;
    }
    // position for a value that follows on the line of the last item
    inline void valueSameLine() {
        pendingSame() = false;
        const float sp = ImGui::GetStyle().ItemSpacing.x;
        // the label must still be the last item (raw ImGui items in between: no column)
        bool lastIsLabel = labelPending() && ImGui::GetItemRectMax().x == labelEndX() && ImGui::GetItemRectMin().x == labelStartX();
        labelPending() = false;
        if (!lastIsLabel) { ImGui::SameLine(); return; }
        const LabelCol& c = labelCols()[ImGui::GetID("##tw_labelcol")];
        float x0 = ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMin().x;
        float full = rightEdge() - x0;
        float col = labelStartX() + (std::min)(c.maxW + sp, 0.45f * full);
        float minVal = ImGui::GetFontSize() * 7.0f;
        if (rightEdge() - col < minVal) {
            // too narrow for a value column: values on a fixed line below
            ImGui::SetCursorScreenPos(ImVec2(labelStartX() + ImGui::GetFontSize(), ImGui::GetCursorScreenPos().y));
            return;
        }
        if (col < labelEndX() + sp) { ImGui::SameLine(); return; }   // longer label than the column
        ImGui::SameLine(0, 0);
        ImGui::SetCursorScreenPos(ImVec2(col, ImGui::GetCursorScreenPos().y));
    }

    // controls after an item: same line if they fit, with a hysteresis of one
    // character per control (keyed by its unique label) so a changing value in
    // front of them cannot make them jump back and forth
    inline std::unordered_map<ImGuiID, bool>& ctrlWrap() { static std::unordered_map<ImGuiID, bool> m; return m; }
    inline bool SameLineForCtrl(const char* id, float w) {
        pendingSame() = false;
        labelPending() = false;
        float x = ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x;
        bool& wrapped = ctrlWrap()[ImGui::GetID(id)];
        float margin = wrapped ? ImGui::GetFontSize() * 2.0f : 0.0f;
        bool fits = x + w + margin <= rightEdge();
        wrapped = !fits;
        if (fits) { ImGui::SameLine(); return true; }
        if (ImGui::GetCursorScreenPos().x + ImGui::GetFontSize() + w <= rightEdge()) {
            ImGui::SetCursorPosX(ImGui::GetCursorPosX() + ImGui::GetFontSize());
        }
        return false;
    }
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
            // multi-line (static explanations) wrap; single lines are clipped with
            // "..." and the full text as tooltip, so nothing below them moves
            if (!cellMode() && strchr(buf, '\n')) { ImGui::PushTextWrapPos(0.0f); wrap = true; return; }
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

    inline void flushPending(const void* key, float w) {
        (void)key; (void)w;
        if (pendingSame()) { valueSameLine(); }
        else { labelPending() = false; }
    }
    // drop-in replacements for ImGui::Text*: behave identically unless a
    // SameNext() is pending; too long texts wrap (cells: clip with "...")
    inline void TextColored(const ImVec4& col, const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        flushPending(fmt, textWidth(buf));
        FitText f(buf, sizeof(buf));
        ImGui::TextColored(col, "%s", buf);
        f.done();
        if (!cellMode()) { noteLabel(buf); }
    }
    inline void Text(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        flushPending(fmt, textWidth(buf));
        FitText f(buf, sizeof(buf));
        ImGui::TextUnformatted(buf);
        f.done();
        if (!cellMode()) { noteLabel(buf); }
    }
    inline void TextU(const char* text) { Text("%s", text); }
    inline void TextDisabled(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        flushPending(fmt, textWidth(buf));
        FitText f(buf, sizeof(buf));
        ImGui::TextDisabled("%s", buf);
        f.done();
        if (!cellMode()) { noteLabel(buf); }
    }

    inline void SameTextColored(const ImVec4& col, const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        valueSameLine();
        FitText f(buf, sizeof(buf));
        ImGui::TextColored(col, "%s", buf);
        f.done();
    }
    inline void SameText(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        valueSameLine();
        FitText f(buf, sizeof(buf));
        ImGui::TextUnformatted(buf);
        f.done();
    }
    inline void SameTextDisabled(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        valueSameLine();
        FitText f(buf, sizeof(buf));
        ImGui::TextDisabled("%s", buf);
        f.done();
    }
    inline bool SameCheckbox(const char* label, bool* v) {
        SameLineForCtrl(label, checkboxWidth(label));
        return ImGui::Checkbox(label, v);
    }
    inline bool SameSmallButton(const char* label) {
        SameLineForCtrl(label, smallButtonWidth(label));
        return ImGui::SmallButton(label);
    }
    inline bool SameButton(const char* label, const ImVec2& size = ImVec2(0, 0)) {
        SameLineForCtrl(label, size.x > 0 ? size.x : buttonWidth(label));
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
