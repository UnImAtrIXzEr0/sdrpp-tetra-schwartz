#include <gui/gui.h>

namespace gui {
    MobileMainWindow pmainWindow;
    MobileMainWindow &mainWindow = pmainWindow;
    ImGui::WaterFall waterfall;
    FrequencySelect freqSelect;
    ThemeManager themeManager;
    Menu menu;
};