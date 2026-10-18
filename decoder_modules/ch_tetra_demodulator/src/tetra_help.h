#pragma once
// Central help texts of the measurement values: what it is, how the program
// measures it, the standard and its limit, and whether the value is a test
// per the standard or indicative. Used for the tooltips (tap on touch screens).

#include <imgui.h>

namespace tetra_help {
    enum Key { LEVEL, RF_STATUS, MER, COUNT };

    inline const char* text(Key k) {
        switch (k) {
        case LEVEL:
            return "Level of the TETRA channel (mean power through the channel filter).\n"
                   "How: dBFS = relative to the ADC full scale; dBm only with 'Level cal.'\n"
                   "(remembers the gain, later gain steps corrected from the gain table, +-1 dB).\n"
                   "Standard: EN 300 394-1 7.1.1 measures transmitter power at the antenna\n"
                   "connector (+-2 dB). Over the air this is a field level: indicative.";
        case RF_STATUS:
            return "ADC and level check: overload of the ADC by the TETRA signal or by any\n"
                   "other signal in the SDR band, level too high or too low.\n"
                   "Measurements are only valid without overload (receiver linearity).";
        case MER:
            return "Modulation error ratio and decoder EVM: quick values from the decoder,\n"
                   "measured after its tracking loops (phase, frequency, gain) - too optimistic.\n"
                   "Not a value of the TETRA standard (MER comes from broadcasting).\n"
                   "Per standard: 'Modulation accuracy (EN 300 394-1, 10.1.3)' below.";
        default:
            return "";
        }
    }

    // tooltip for the item just drawn (on touch screens shown on tap)
    inline void tip(Key k) {
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", text(k)); }
    }
}
