#pragma once

#include <set>
#include <string>
#include <vector>

namespace sourcemenu {
    void init();
    void draw(void* ctx);

    // Sources hidden from the source list (config "hiddenSources"); the source
    // in use is always shown. Managed in the Module Manager.
    bool isSourceHidden(const std::string& name);
    void setSourceHidden(const std::string& name, bool hidden);
    std::vector<std::string> allSourceNames();
}