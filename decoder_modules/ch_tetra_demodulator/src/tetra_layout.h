#pragma once
// Layout helpers with an instrument-style value column and segment memory
// (see placeSegment): nothing is cut off, nothing moves at a constant width.
#include <imgui.h>
#include <imgui_internal.h>
#include <cmath>
#include <cstdint>
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

    // Continue on the line of the last item if 'w' more pixels fit, else on a
    // continuation line (segment memory, see placeSegment)
    inline bool SameLineFor(float w);

    inline float textWidth(const char* s) { return ImGui::CalcTextSize(s, nullptr, true).x; }

    // ---- Instrument-style layout (v0.23.6) ----
    // Nothing is cut off and nothing moves while the window width stays the same:
    //  - A "Label:" line puts its first value into a value column at a fixed x
    //    (widest label of the window, at most 45 % of the width; below 7 em of
    //    room the values go to a fixed line below the label).
    //  - Every further part of a line (value, unit, control) is a "segment" with
    //    its own memory, keyed by the line (its label / first text) and the
    //    segment's number within the line - not by its position in the window,
    //    so lines that come and go elsewhere cannot disturb it, and a value drawn
    //    with different texts (e.g. timeslot content) keeps its place.
    //  - A static sub-label in front of a value (" peak", " | Frame:") moves to
    //    the continuation line together with its value, a static unit after a
    //    value (" MHz ", " dB") together with the value in front of it.
    //  - A segment reserves the widest text it has shown so far, so a value
    //    getting longer does not push the rest of the line.
    //  - A segment that does not fit goes to a continuation line (value column,
    //    full text). At a constant window width this decision is frozen - it can
    //    only change once if a value grows beyond the room (then it wraps and
    //    stays wrapped). When the width changes it is decided again, with a
    //    hysteresis of 2 em before a wrapped segment comes back.
    //  - Text too long for a whole line wraps word by word and keeps the largest
    //    height it had at this width (no line count flicker).
    // Table cells (CellScope) stay single line, clipped with "..." and the full
    // text as tooltip, so rows keep their height; the columns can be resized.
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
    inline float contentLeft() { return ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMin().x; }
    inline float fullWidth() { return rightEdge() - contentLeft(); }

    // memory of one segment
    struct Seg {
        float maxW = 0.0f;      // widest text shown at this window width
        float atW = -1.0f;      // window width the decision was made for
        bool wrapped = false;
    };
    inline std::unordered_map<ImGuiID, Seg>& segs() { static std::unordered_map<ImGuiID, Seg> m; return m; }
    // the line being drawn
    struct LineState {
        ImGuiID key = 0;
        ImGuiWindow* win = nullptr;
        float contX = 0.0f;         // x of continuation lines
        float lastStart = 0.0f;     // last segment: start x, reserved width, y
        float lastResW = 0.0f;
        float lastY = 0.0f;
        int nSeg = 0;               // segments so far in this line
        ImGuiID prevId = 0;         // previous segment of the line
        bool prevGlue = false;      // ... was a static sub-label
        bool prevValue = false;     // ... was a value (not static text)
        bool prevSameLine = false;  // ... and is directly in front of the current one
    };
    inline LineState& line() { static LineState l; return l; }
    inline void beginLine(const char* key) {
        LineState& l = line();
        l.key = ImGui::GetID(key);
        l.win = ImGui::GetCurrentWindowRead();
        l.nSeg = 0;
        l.lastResW = 0.0f;
        l.prevId = 0;
        l.prevGlue = false;
    }
    // key of a segment: line + its number within the line
    inline ImGuiID segKey(const void* fmt) {
        (void)fmt;
        LineState& l = line();
        int n = l.nSeg++;
        return ImHashData(&n, sizeof(n), l.key);
    }
    // sub-label: plain or disabled text without conversions (" peak", " | Frame:")
    inline bool& nextIsGlue() { static bool g = false; return g; }
    inline bool isGlue(const void* fmt, bool keyIsId) {
        bool g = nextIsGlue();
        nextIsGlue() = false;
        return g && !keyIsId && fmt && !strchr((const char*)fmt, '%');
    }
    // a static unit belongs to the value in front of it
    inline bool isUnit(const char* t) {
        static const char* units[] = { "MHz", "kHz", "Hz", "dB", "dBm", "dBFS", "dBc", "%", "ppm", "s", "ms", "deg", "S/s", "/" };
        while (*t == ' ') { t++; }
        size_t n = strlen(t);
        while (n > 0 && t[n - 1] == ' ') { n--; }
        for (const char* u : units) { if (strlen(u) == n && strncmp(t, u, n) == 0) { return true; } }
        return false;
    }
    // after a line-start text: remember it as the line, and as label if it is one
    inline void noteLineStart(const char* t, const char* key) {
        labelPending() = false;
        beginLine(key);
        LineState& l = line();
        l.contX = ImGui::GetItemRectMin().x + ImGui::GetFontSize();
        l.lastStart = ImGui::GetItemRectMin().x;
        l.lastResW = ImGui::GetItemRectSize().x;
        l.lastY = ImGui::GetItemRectMin().y;
        if (!isLabel(t)) { return; }
        LabelCol& c = labelCols()[ImGui::GetID("##tw_labelcol")];
        float w = ImGui::GetItemRectSize().x;
        if (w > c.maxW) { c.maxW = w; }
        labelPending() = true;
        labelStartX() = ImGui::GetItemRectMin().x;
        labelEndX() = ImGui::GetItemRectMax().x;
    }

    // Place the next segment (width w) after the last item. Returns true if it
    // stays on the same line. 'key' identifies the segment (format string or
    // control label).
    inline bool placeSegment(const void* key, float w, bool keyIsId = false) {
        pendingSame() = false;
        bool glue = isGlue(key, keyIsId);
        const bool unit = glue && isUnit((const char*)key);
        if (unit) { glue = false; }
        const ImGuiStyle& st = ImGui::GetStyle();
        const float sp = st.ItemSpacing.x;
        const float em = ImGui::GetFontSize();
        const float right = rightEdge();
        const float full = fullWidth();
        LineState& l = line();
        if (l.win != ImGui::GetCurrentWindowRead()) { beginLine("##tw_noline"); l.contX = contentLeft() + em; l.lastResW = 0.0f; }

        // where the segment would start on this line
        bool lastIsLabel = labelPending() && ImGui::GetItemRectMax().x == labelEndX() && ImGui::GetItemRectMin().x == labelStartX();
        labelPending() = false;
        float y = ImGui::GetItemRectMin().y;
        float x;
        bool firstValue = false;
        if (lastIsLabel) {
            const LabelCol& c = labelCols()[ImGui::GetID("##tw_labelcol")];
            float col = labelStartX() + (std::min)(c.maxW + sp, 0.45f * full);
            if (right - col < em * 7.0f) {
                // too narrow for a value column: all values on lines below the label
                l.contX = labelStartX() + em;
                ImGui::SetCursorScreenPos(ImVec2(l.contX, ImGui::GetCursorScreenPos().y));
                l.lastStart = l.contX; l.lastResW = 0.0f; l.lastY = ImGui::GetCursorScreenPos().y;
                // the following segments continue on that line
                ImGuiID nid = keyIsId ? (ImGuiID)(uintptr_t)key : segKey(key);
                Seg& s = segs()[nid];
                s.maxW = (std::max)(s.maxW, w);
                l.lastResW = s.maxW;
                l.prevId = nid; l.prevGlue = glue; l.prevValue = !glue && !unit && !keyIsId; l.prevSameLine = true;
                return false;
            }
            x = (std::max)(col, labelEndX() + sp);
            l.contX = col;
            firstValue = true;
        }
        else if (ImGui::GetItemRectMin().x == l.lastStart && y == l.lastY && l.lastResW > 0.0f) {
            x = l.lastStart + l.lastResW + sp;   // after the reserved width of the previous segment
        }
        else {
            x = ImGui::GetItemRectMax().x + sp;  // after a raw ImGui item
            l.prevSameLine = false;
        }
        bool prevDirect = !lastIsLabel && l.prevId != 0 && l.prevSameLine;

        ImGuiID id = keyIsId ? (ImGuiID)(uintptr_t)key : segKey(key);
        Seg& s = segs()[id];
        bool widthChanged = std::fabs(full - s.atW) > 0.5f;
        if (widthChanged) { s.maxW = w; }
        else { s.maxW = (std::max)(s.maxW, w); }
        bool fits = x + s.maxW <= right + 0.5f;
        if (widthChanged) {
            // new width: decide again, a wrapped segment needs 2 em extra to come back
            s.wrapped = s.wrapped && s.atW >= 0.0f ? !(x + s.maxW + 2.0f * em <= right) : !fits;
            s.atW = full;
        }
        else if (!s.wrapped && !fits) {
            s.wrapped = true;   // grew beyond the room: wrap (and stay wrapped at this width)
        }
        // never leave the line if the continuation line would not have more room
        if (s.wrapped && x <= l.contX + 0.5f) { s.wrapped = false; }
        bool drawWrapped = s.wrapped;
        // a value that has to wrap takes its static sub-label along: from the next
        // frame on the sub-label starts the continuation line and the value follows it
        if (s.wrapped && prevDirect && (l.prevGlue || (unit && l.prevValue))) {
            Seg& ps = segs()[l.prevId];
            if (!ps.wrapped) { ps.wrapped = true; s.wrapped = false; }
        }

        if (!drawWrapped) {
            ImGui::SameLine(0, 0);
            ImGui::SetCursorScreenPos(ImVec2(x, ImGui::GetCursorScreenPos().y));
            l.lastStart = x; l.lastY = y;
        }
        else {
            ImGui::SetCursorScreenPos(ImVec2(l.contX, ImGui::GetCursorScreenPos().y));
            l.lastStart = l.contX; l.lastY = ImGui::GetCursorScreenPos().y;
        }
        l.lastResW = s.maxW;
        l.prevId = id; l.prevGlue = glue; l.prevValue = !glue && !unit && !keyIsId; l.prevSameLine = true;
        (void)firstValue;
        return !drawWrapped;
    }
    inline bool valueSameLine(const void* key, float w) { return placeSegment(key, w); }
    // a separator "|" is not drawn at the start of a continuation line
    inline const char* dropLeadingBar(const char* t, bool sameLine) {
        if (sameLine) { return t; }
        const char* p = t;
        while (*p == ' ') { p++; }
        if (*p != '|') { return t; }
        p++;
        while (*p == ' ') { p++; }
        return p;
    }

    // controls after an item: segment keyed by the control's unique label
    inline bool SameLineForCtrl(const char* id, float w) {
        return placeSegment((const void*)(uintptr_t)ImGui::GetID(id), w, true);
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

    // Fit 'buf' into the remaining width. Cells: clip with "..." (tooltip with
    // the full text). Elsewhere nothing is cut: text longer than the room wraps
    // word by word and keeps the largest height it had at this window width.
    struct WrapMem { float maxH = 0.0f; float atW = -1.0f; };
    inline std::unordered_map<ImGuiID, WrapMem>& wrapMems() { static std::unordered_map<ImGuiID, WrapMem> m; return m; }
    struct FitText {
        bool wrap = false, cut = false;
        ImGuiID memId = 0;
        float y0 = 0.0f;
        char full[512];
        FitText(char* buf, size_t n, const void* key = nullptr) {
            float avail = ImGui::GetContentRegionAvail().x;
            if (!cellMode()) {
                // wrap only if needed; the height memory keeps the space if a
                // shorter value later fits on fewer lines
                memId = ImHashData(&key, sizeof(key), line().key ^ ImGui::GetID("##tw_wrap"));
                y0 = ImGui::GetCursorScreenPos().y;
                if (textWidth(buf) > avail && avail > 0) {
                    ImGui::PushTextWrapPos(0.0f);
                    wrap = true;
                }
                return;
            }
            if (textWidth(buf) <= avail || avail <= 0) { return; }
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
            if (wrap) { ImGui::PopTextWrapPos(); }
            if (cut && ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", full); }
            if (cellMode() || !memId) { return; }
            WrapMem& m = wrapMems()[memId];
            float full = fullWidth();
            float h = ImGui::GetItemRectSize().y;
            if (std::fabs(full - m.atW) > 0.5f) { m.atW = full; m.maxH = h; }
            if (h > m.maxH) { m.maxH = h; }
            if (h + 0.5f < m.maxH) {
                // keep the room of the taller version (only below a single item line)
                ImVec2 c = ImGui::GetCursorScreenPos();
                ImGui::SetCursorScreenPos(ImVec2(c.x, c.y + (m.maxH - h)));
            }
            (void)y0;
        }
    };

    // a line-start text after SameNext() is a segment of the previous line
    inline bool flushPending(const void* key, float w) {
        if (pendingSame()) { valueSameLine(key, w); return true; }
        labelPending() = false;
        return false;
    }
    // drop-in replacements for ImGui::Text*
    inline void TextColored(const ImVec4& col, const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        bool seg = flushPending(fmt, textWidth(buf));
        FitText f(buf, sizeof(buf), fmt);
        ImGui::TextColored(col, "%s", buf);
        f.done();
        if (!cellMode() && !seg) { noteLineStart(buf, fmt); }
    }
    inline void Text(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        bool seg = flushPending(fmt, textWidth(buf));
        FitText f(buf, sizeof(buf), fmt);
        ImGui::TextUnformatted(buf);
        f.done();
        // TextU passes "%s": key the line by its (static) text then
        if (!cellMode() && !seg) { noteLineStart(buf, strcmp(fmt, "%s") == 0 ? buf : fmt); }
    }
    inline void TextU(const char* text) { Text("%s", text); }
    inline void TextDisabled(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        bool seg = flushPending(fmt, textWidth(buf));
        FitText f(buf, sizeof(buf), fmt);
        ImGui::TextDisabled("%s", buf);
        f.done();
        if (!cellMode() && !seg) { noteLineStart(buf, fmt); }
    }

    inline void SameTextColored(const ImVec4& col, const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        if (cellMode()) { ImGui::SameLine(); } else { valueSameLine(fmt, textWidth(buf)); }
        FitText f(buf, sizeof(buf), fmt);
        ImGui::TextColored(col, "%s", buf);
        f.done();
    }
    inline void SameText(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        bool same = true;
        if (cellMode()) { ImGui::SameLine(); } else { nextIsGlue() = true; same = valueSameLine(fmt, textWidth(buf)); }
        if (!same) { memmove(buf, dropLeadingBar(buf, false), strlen(dropLeadingBar(buf, false)) + 1); }
        FitText f(buf, sizeof(buf), fmt);
        ImGui::TextUnformatted(buf);
        f.done();
    }
    inline void SameTextDisabled(const char* fmt, ...) {
        char buf[512];
        va_list ap; va_start(ap, fmt); vfmt(buf, sizeof(buf), fmt, ap); va_end(ap);
        bool same = true;
        if (cellMode()) { ImGui::SameLine(); } else { nextIsGlue() = true; same = valueSameLine(fmt, textWidth(buf)); }
        if (!same) { memmove(buf, dropLeadingBar(buf, false), strlen(dropLeadingBar(buf, false)) + 1); }
        FitText f(buf, sizeof(buf), fmt);
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

    inline bool SameLineFor(float w) {
        static const char key[] = "##tw_samelinefor%";   // "%": not a sub-label
        return placeSegment(key, w);
    }
}
