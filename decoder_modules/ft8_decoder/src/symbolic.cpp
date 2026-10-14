
#include <fstream>
#include <utils/strings.h>
#include "symbolic.h"



bool isSignalStrength(const std::string &maybeStrength) {
    if (maybeStrength.length() < 2) return false;
    if (maybeStrength[0] != '+' && maybeStrength[0] != '-') return false;
    for (int i = 1; i < maybeStrength.length(); i++) {
        if (!std::isdigit(maybeStrength[i])) return false;
    }
    return true;
}

bool isLocator(const std::string &maybeLoc) {
    if (maybeLoc.length() != 4) return false;
    if (std::isupper(maybeLoc[0]) && std::isupper(maybeLoc[1]) && std::isalpha(maybeLoc[0]) && std::isalpha(maybeLoc[1]) && std::isdigit(maybeLoc[2]) && std::isdigit(maybeLoc[3])) {
        return true;
    }
    return false;
}

std::string extractCallsignFromFT8(const std::string &message) {
    std::vector<std::string> parts;
    splitString(message," ",[&](const std::string &str) {
        if (!str.empty()) {
            parts.emplace_back(str);
        }
    });
    if (parts.empty()) {
        return "";
    }
    auto &lastPart = parts.back();
    if (parts.size() > 1 && (lastPart == "RR73" || lastPart == "RRR" || lastPart == "73" || isLocator(lastPart))) {       /// .... CALL KO80
        return parts[parts.size()-2];
    }
    if (lastPart.size() > 2 && (lastPart[0] == 'R' && isSignalStrength(lastPart.substr(1)) || isSignalStrength(lastPart))) {        // DXCALL CALL R+10
        return parts[parts.size()-2];
    }
    if (parts.size() == 2 && parts[0] == "CQ") {    // cq without loc
        return parts[1];
    }

    return "";
}




