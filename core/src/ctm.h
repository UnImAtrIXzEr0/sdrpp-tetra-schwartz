#pragma once

#include <chrono>

inline long long currentTimeMillis() {
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
    long long msec = std::chrono::time_point_cast<std::chrono::milliseconds>(t1).time_since_epoch().count();
    return msec;
}

inline long long currentTimeNanos() {
#ifdef __linux__
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
    long long msec = std::chrono::time_point_cast<std::chrono::nanoseconds>(t1).time_since_epoch().count();
    return msec;
//    timespec time1;
//    clock_gettime(CLOCK_MONOTONIC_COARSE, &time1);
//    return time1.tv_nsec + time1.tv_sec * 1000000000L;
#else
    std::chrono::system_clock::time_point t1 = std::chrono::system_clock::now();
    long long msec = std::chrono::time_point_cast<std::chrono::nanoseconds>(t1).time_since_epoch().count();
    return msec;
#endif
}

