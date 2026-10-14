#pragma once
#include <gui/widgets/waterfall.h>
#include <gui/widgets/frequency_select.h>
#include <gui/widgets/menu.h>
#include <gui/dialogs/loading_screen.h>
#include <module.h>
#include <gui/brown/mobile_main_window.h>
#include <gui/theme_manager.h>

namespace gui {
    SDRPP_EXPORT ImGui::WaterFall waterfall;
    SDRPP_EXPORT FrequencySelect freqSelect;
    SDRPP_EXPORT Menu menu;
    SDRPP_EXPORT ThemeManager themeManager;
    SDRPP_EXPORT MobileMainWindow& mainWindow;

    struct VFOFrequencyChange {
//        WaterfallVFO* vfo;
        double freq;
    };

    SDRPP_EXPORT Event<VFOFrequencyChange> vfoFrequencyChanged;


    void selectSource(std::string name);
};