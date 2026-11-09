#pragma once
// HTTP User-Agent of this edition for all downloads (RadioID, BrandMeister,
// map tiles). Tile servers such as openstreetmap.org require a unique agent
// that names the application and gives a contact URL.

#include <string>
#include <version.h>

inline std::string aiaUserAgent() {
    return std::string("SDRPP-AIA/") + EDITION_VERSION + " (+https://github.com/UnImAtrIXzEr0/sdrpp-air-interface-analyzer)";
}
