#include <gui/menus/module_manager.h>
#include <imgui.h>
#include <core.h>
#include <string.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <gui/dialogs/dialog_box.h>

namespace module_manager_menu {
    char modName[1024];
    std::vector<std::string> modTypes;
    std::string toBeRemoved;
    std::string modTypesTxt;
    std::string errorMessage;
    int modTypeId;
    bool confirmOpened = false;
    bool errorOpen = false;

    void init() {
        modName[0] = 0;

        modTypes.clear();
        modTypesTxt = "";
        for (auto& [name, mod] : core::moduleManager.modules) {
            modTypes.push_back(name);
            modTypesTxt += name;
            modTypesTxt += '\0';
        }
        modTypeId = 0;
    }

    void draw(void* ctx) {
        bool modified = false;

        // Calculate delete button size and cell size
        ImVec2 cellpad = ImGui::GetStyle().CellPadding;
        float lheight = ImGui::GetTextLineHeight();
        float cellWidth = lheight - (2.0f * cellpad.y);
        float hdiff = cellpad.x - cellpad.y;
        ImVec2 btnSize = ImVec2(lheight, lheight - 1);
        ImVec2 textOff = ImVec2(3.0f * style::uiScale, -5.0f * style::uiScale);

        bool hidChanged = false;
        if (ImGui::BeginTable("Module Manager Table", 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY, ImVec2(0, 260.0f * style::uiScale))) {
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("Type");
            ImGui::TableSetupColumn("Menu", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("On", ImGuiTableColumnFlags_WidthFixed);
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, cellWidth);
            ImGui::TableSetupScrollFreeze(5, 1);
            ImGui::TableHeadersRow();

            for (auto& [name, inst] : core::moduleManager.instances) {
                ImGui::TableNextRow();

                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(name.c_str());

                ImGui::TableSetColumnIndex(1);
                ImGui::TextUnformatted(inst.module.info->name);

                // shown in the menu bar on the left
                ImGui::TableSetColumnIndex(2);
                {
                    bool shown = !gui::menu.isHidden(name);
                    if (ImGui::Checkbox(("##module_mgr_menu_" + name).c_str(), &shown)) {
                        if (shown) { gui::menu.hidden.erase(name); }
                        else { gui::menu.hidden.insert(name); }
                        hidChanged = true;
                    }
                    if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Show this module in the menu bar"); }
                }

                // enabled (running)
                ImGui::TableSetColumnIndex(3);
                {
                    bool en = inst.instance->isEnabled();
                    if (ImGui::Checkbox(("##module_mgr_on_" + name).c_str(), &en)) {
                        en ? inst.instance->enable() : inst.instance->disable();
                        modified = true;
                    }
                    if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Module enabled"); }
                }

                ImGui::TableSetColumnIndex(4);
                ImVec2 cpos = ImGui::GetCursorPos();
                ImGui::SetCursorPos(ImVec2(cpos.x - hdiff, cpos.y + 1));
                if (ImGui::Button(("##module_mgr_" + name).c_str(), btnSize)) {
                    toBeRemoved = name;
                    confirmOpened = true;
                }
                ImGui::SetCursorPos(ImVec2(cpos.x + textOff.x, cpos.y + textOff.y));
                ImGui::TextUnformatted("_");
            }
            ImGui::EndTable();
        }

        if (ImGui::GenericDialog("module_mgr_confirm_", confirmOpened, GENERIC_DIALOG_BUTTONS_YES_NO, []() {
                ImGui::Text("Deleting \"%s\". Are you sure?", toBeRemoved.c_str());
            }) == GENERIC_DIALOG_BUTTON_YES) {
            core::moduleManager.deleteInstance(toBeRemoved);
            modified = true;
        }

        ImGui::GenericDialog("module_mgr_error_", errorOpen, GENERIC_DIALOG_BUTTONS_OK, []() {
            ImGui::TextUnformatted(errorMessage.c_str());
        });

        // Add module row with slightly different settings
        if (ImGui::BeginTable("Module Manager Add Table", 3)) {
            ImGui::TableSetupColumn("Name");
            ImGui::TableSetupColumn("Type");
            ImGui::TableSetupColumn("", ImGuiTableColumnFlags_WidthFixed, cellWidth + cellpad.x);
            
            ImGui::TableNextRow();

            ImGui::TableSetColumnIndex(0);
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x + cellpad.x);
            ImGui::InputText("##module_mod_name", modName, 1000);

            ImGui::TableSetColumnIndex(1);
            ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x + cellpad.x);
            ImGui::Combo("##module_mgr_type", &modTypeId, modTypesTxt.c_str());

            ImGui::TableSetColumnIndex(2);
            if (strlen(modName) == 0) { style::beginDisabled(); }
            if (ImGui::Button("+##module_mgr_add_btn", ImVec2(btnSize.x, 0))) {
                if (!core::moduleManager.createInstance(modName, modTypes[modTypeId])) {
                    core::moduleManager.postInit(modName);
                    modified = true;
                }
                else {
                    errorMessage = "Could not create new instance of " + modTypes[modTypeId];
                    errorOpen = true;
                }
            }
            if (strlen(modName) == 0) { style::endDisabled(); }
            ImGui::EndTable();
        }

        // Built-in menus that can be hidden from the menu bar
        ImGui::TextUnformatted("Built-in menus in the menu bar:");
        {
            static const char* builtins[] = { "VFO Color", "Debug", "Band Plan", "Display", "Theme" };
            for (int i = 0; i < (int)(sizeof(builtins) / sizeof(builtins[0])); i++) {
                bool shown = !gui::menu.isHidden(builtins[i]);
                if (i > 0) {
                    // same line only if the box still fits, else wrap
                    float w = ImGui::GetFrameHeight() + ImGui::GetStyle().ItemInnerSpacing.x + ImGui::CalcTextSize(builtins[i]).x;
                    float x = ImGui::GetItemRectMax().x + ImGui::GetStyle().ItemSpacing.x;
                    if (x + w <= ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x) { ImGui::SameLine(); }
                }
                if (ImGui::Checkbox((std::string(builtins[i]) + "##module_mgr_show_" + std::to_string(i)).c_str(), &shown)) {
                    if (shown) { gui::menu.hidden.erase(builtins[i]); }
                    else { gui::menu.hidden.insert(builtins[i]); }
                    hidChanged = true;
                }
            }
        }
        ImGui::TextDisabled("Menu: shown in the menu bar on the left. On: module running.");
        if (hidChanged) {
            core::configManager.acquire();
            json arr = json::array();
            for (auto& h : gui::menu.hidden) { arr.push_back(h); }
            core::configManager.conf["hiddenMenus"] = arr;
            core::configManager.release(true);
        }

        if (modified) {
            // Update enabled and disabled modules
            core::configManager.acquire();
            json instances;
            for (auto [_name, inst] : core::moduleManager.instances) {
                instances[_name]["module"] = inst.module.info->name;
                instances[_name]["enabled"] = inst.instance->isEnabled();
            }
            core::configManager.conf["moduleInstances"] = instances;
            core::configManager.release(true);
        }
    }
}
