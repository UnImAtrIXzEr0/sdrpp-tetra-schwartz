#pragma once
// TETRA-SCHWARTZ touch mode helpers (Display -> Touch mode): drag with the
// finger on an empty spot of a window to scroll it, larger touch margins.
#include <imgui.h>
#include <gui/menus/display.h>

namespace touch {
    inline bool enabled() { return displaymenu::phoneLayout; }

    // call inside a window/child after its content: a drag that started on an
    // empty spot (no item hovered or active) scrolls the window
    inline void dragScroll() {
        if (!enabled()) { return; }
        static ImGuiID owner = 0;
        ImGuiID me = ImGui::GetID("##touch_scroll");
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
            owner = (ImGui::IsWindowHovered(ImGuiHoveredFlags_ChildWindows) && !ImGui::IsAnyItemHovered() && !ImGui::IsAnyItemActive()) ? me : (owner == me ? 0 : owner);
        }
        if (owner == me && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            float dy = ImGui::GetIO().MouseDelta.y;
            if (dy != 0.0f) { ImGui::SetScrollY(ImGui::GetScrollY() - dy); }
        }
        if (!ImGui::IsMouseDown(ImGuiMouseButton_Left) && owner == me) { owner = 0; }
    }
}
