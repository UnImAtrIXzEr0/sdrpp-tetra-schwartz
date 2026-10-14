#include <gui/dialogs/credits.h>
#include <imgui.h>
#include <gui/icons.h>
#include <gui/style.h>
#include <config.h>
#include <credits.h>
#include <version.h>

namespace credits {
    ImFont* bigFont;
    ImVec2 imageSize(128.0f, 128.0f);

    void init() {
        imageSize = ImVec2(128.0f * style::uiScale, 128.0f * style::uiScale);
    }

    void show() {
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(20.0f, 20.0f));
        ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
        ImVec2 dispSize = ImGui::GetIO().DisplaySize;
        ImVec2 center = ImVec2(dispSize.x / 2.0f, dispSize.y / 2.0f);
        ImGui::SetNextWindowPos(center, ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        // never larger than the screen (scrolls instead)
        ImGui::SetNextWindowSizeConstraints(ImVec2(0, 0), ImVec2(dispSize.x * 0.96f, dispSize.y * 0.96f));
        ImGui::OpenPopup("Credits");
        ImGui::BeginPopupModal("Credits", NULL, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove);

        // Header: edition name and logo
        ImGui::BeginGroup();
        ImGui::PushFont(style::bigFont);
        ImGui::TextUnformatted(EDITION_NAME);
        ImGui::PopFont();
        ImGui::Text("Version %s  -  experimental beta for amateur radio TETRA", EDITION_VERSION);
        ImGui::Spacing();
        ImGui::TextUnformatted("A modified version (fork) of SDR++ by Alexandre Rouma (ON5RYZ),");
        ImGui::TextUnformatted("revised and extended for amateur radio TETRA by UnImAtrIXzEr0.");
        ImGui::TextDisabled("Not an official release of SDR++ or SDR++Brown and not endorsed by their authors.");
        ImGui::TextDisabled("Please report problems with this edition only at github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz/issues");
        ImGui::EndGroup();
        ImGui::SameLine(0.0f, 40.0f * style::uiScale);
        ImGui::Image(icons::LOGO, imageSize);
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Origin of the code, with authors and licences
        ImGui::TextUnformatted("Origin");
        if (ImGui::BeginTable("##credits_origin", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
            ImGui::TableSetupColumn("Part");
            ImGui::TableSetupColumn("Authors / copyright holders");
            ImGui::TableSetupColumn("Licence, source");
            ImGui::TableHeadersRow();
            static const char* rows[][3] = {
                { "SDR++ (original software)", "Alexandre Rouma (ON5RYZ) and the SDR++ contributors", "GPL-3.0, github.com/AlexandreRouma/SDRPlusPlus" },
                { "SDR++Brown v" VERSION_STR " (code base)", "sannysanoff and contributors", "GPL-3.0, github.com/sannysanoff/SDRPlusPlusBrown" },
                { "TETRA demodulator plugin", "cropinghigh", "GPL-3.0, github.com/cropinghigh/sdrpp-tetra-demodulator" },
                { "osmo-tetra (TETRA decoder)", "Harald Welte, Sylvain Munaut, Holger H. P. Freyther and contributors (Osmocom)", "AGPL-3.0, gitea.osmocom.org/tetra/osmo-tetra" },
                { "TETRA speech codec", "ETSI (reference code of EN 300 395-2)", "ETSI terms, downloaded from etsi.org at build time" },
                { "Modifications of this edition", "(C) 2026 UnImAtrIXzEr0", "GPL-3.0, github.com/UnImAtrIXzEr0/sdrpp-tetra-schwartz" },
            };
            for (auto& r : rows) {
                ImGui::TableNextRow();
                for (int c = 0; c < 3; c++) {
                    ImGui::TableSetColumnIndex(c);
                    ImGui::TextUnformatted(r[c]);
                }
            }
            ImGui::EndTable();
        }
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();

        // Original SDR++ credits, unchanged
        ImGui::TextUnformatted("SDR++ is brought to you by Alexandre Rouma (ON5RYZ) with the help of\n\n");

        ImGui::Columns(4, "CreditColumns", true);

        ImGui::TextUnformatted("Contributors");
        for (int i = 0; i < sdrpp_credits::contributorCount; i++) {
            ImGui::BulletText("%s", sdrpp_credits::contributors[i]);
        }

        ImGui::NextColumn();
        ImGui::TextUnformatted("Libraries");
        for (int i = 0; i < sdrpp_credits::libraryCount; i++) {
            ImGui::BulletText("%s", sdrpp_credits::libraries[i]);
        }

        ImGui::NextColumn();
        ImGui::TextUnformatted("Hardware Donators");
        for (int i = 0; i < sdrpp_credits::hardwareDonatorCount; i++) {
            ImGui::BulletText("%s", sdrpp_credits::hardwareDonators[i]);
        }

        ImGui::NextColumn();
        ImGui::TextUnformatted("Patrons");
        for (int i = 0; i < sdrpp_credits::patronCount; i++) {
            ImGui::BulletText("%s", sdrpp_credits::patrons[i]);
        }

        ImGui::Columns(1, "CreditColumnsEnd", true);

        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Spacing();
        ImGui::TextDisabled("Data services (on request): RadioID.net (DMR IDs), BrandMeister (talkgroup names). Map exports: OpenStreetMap (C) OpenStreetMap contributors, Leaflet.");
        ImGui::TextDisabled("Theme names are invented names. This project is not affiliated with any manufacturer and uses no third-party logos.");
        ImGui::TextDisabled("Free software under GPL-3.0 (osmo-tetra parts: AGPL-3.0), WITHOUT ANY WARRANTY. Details and list of changes: NOTICE.md and license.");
        ImGui::TextDisabled(EDITION_NAME " v" EDITION_VERSION " built " __DATE__ " " __TIME__ ".");

        ImGui::EndPopup();
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();
    }
}