#pragma once

#include <utils/flog.h>
#include <string>

inline bool hl2_error(std::string str) {
    flog::error("{0}", str.c_str());
    return false;
}