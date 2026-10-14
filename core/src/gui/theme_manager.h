#pragma once
#include <map>
#include <mutex>
#include <imgui.h>
#include <vector>
#include <json.hpp>

using nlohmann::json;

struct Theme {
    std::string author;
    json data;
};

class ThemeManager {
public:
    bool loadThemesFromDir(std::string path);
    bool loadTheme(std::string path);
    bool applyTheme(std::string name);

    std::vector<std::string> getThemeNames();

    ImVec4 waterfallBg = ImVec4(0.0f, 0.0f, 0.0f, 1.0f);
    ImVec4 fftHoldColor = ImVec4(0.0f, 1.0f, 0.75f, 1.0f);
    ImVec4 clearColor = ImVec4(0.0666f, 0.0666f, 0.0666f, 1.0f);

    // Optional extra colours for plugin displays (keys starting with
    // "Constellation" or "Vfo", e.g. ConstellationBg, VfoBandwidth). Empty when the theme has none.
    std::map<std::string, ImVec4> extraColors;
    bool getExtraColor(const std::string& key, ImVec4& out) const {
        auto it = extraColors.find(key);
        if (it == extraColors.end()) { return false; }
        out = it->second;
        return true;
    }

private:
    static bool decodeRGBA(std::string str, uint8_t out[4]);
    static void maybeInitThemeManager();

    static std::map<std::string, int> IMGUI_COL_IDS;

    std::map<std::string, Theme> themes;
};