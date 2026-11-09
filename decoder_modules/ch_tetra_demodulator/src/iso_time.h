#pragma once
// Timestamps of the CSV logs: ISO 8601 local time with UTC offset
// ("2026-11-09T07:32:00+01:00"), so that tools in other time zones
// (Google Earth, spreadsheets) place the samples correctly.
// parseIsoTime() also reads logs of v0.24.0 and older, which carry
// local time without an offset.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <string>

inline void isoLocalTm(time_t t, struct tm& lt) {
#ifdef _WIN32
    localtime_s(&lt, &t);
#else
    localtime_r(&t, &lt);
#endif
}

inline void isoUtcTm(time_t t, struct tm& gt) {
#ifdef _WIN32
    gmtime_s(&gt, &t);
#else
    gmtime_r(&t, &gt);
#endif
}

// Offset of local time from UTC at time t in seconds (east positive), DST included
inline long isoUtcOffsetSec(time_t t) {
    struct tm lt, gt;
    isoLocalTm(t, lt);
    isoUtcTm(t, gt);
    int dd = lt.tm_yday - gt.tm_yday;
    if (lt.tm_year != gt.tm_year) { dd = (lt.tm_year > gt.tm_year) ? 1 : -1; }
    return dd * 86400L + (lt.tm_hour - gt.tm_hour) * 3600L + (lt.tm_min - gt.tm_min) * 60L + (lt.tm_sec - gt.tm_sec);
}

// "YYYY-MM-DDTHH:MM:SS+HH:MM" (local time)
inline std::string isoLocalTime(time_t t) {
    struct tm lt;
    isoLocalTm(t, lt);
    char ts[40];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &lt);
    long off = isoUtcOffsetSec(t);
    char sign = off < 0 ? '-' : '+';
    off = std::labs(off);
    size_t n = strlen(ts);
    snprintf(ts + n, sizeof(ts) - n, "%c%02ld:%02ld", sign, off / 3600, (off % 3600) / 60);
    return ts;
}

// "YYYY-MM-DDTHH:MM:SSZ" (UTC)
inline std::string isoUtcTime(time_t t) {
    struct tm gt;
    isoUtcTm(t, gt);
    char ts[32];
    strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &gt);
    return ts;
}

// Days since 1970-01-01 of a proleptic Gregorian date (H. Hinnant's days_from_civil)
inline long long isoDaysFromCivil(long long y, unsigned m, unsigned d) {
    y -= m <= 2;
    const long long era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (long long)doe - 719468;
}

// Parses "YYYY-MM-DDTHH:MM:SS" with optional "Z" or "+HH:MM" / "-HH:MM".
// Without an offset the time is taken as local time of this computer.
// Returns 0 if the text is not a timestamp.
inline time_t parseIsoTime(const std::string& s) {
    int Y, M, D, h, mi, sec, n = 0;
    if (sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d%n", &Y, &M, &D, &h, &mi, &sec, &n) != 6) { return 0; }
    if (M < 1 || M > 12 || D < 1 || D > 31) { return 0; }
    const char* p = s.c_str() + n;
    bool hasOff = false;
    long off = 0;
    if (*p == 'Z') { hasOff = true; }
    else if (*p == '+' || *p == '-') {
        int oh = 0, om = 0;
        if (sscanf(p + 1, "%d:%d", &oh, &om) >= 1) {
            hasOff = true;
            off = (oh * 3600L + om * 60L) * (*p == '-' ? -1 : 1);
        }
    }
    if (hasOff) {
        long long days = isoDaysFromCivil(Y, (unsigned)M, (unsigned)D);
        return (time_t)(days * 86400LL + h * 3600LL + mi * 60LL + sec - off);
    }
    struct tm tmv;
    memset(&tmv, 0, sizeof(tmv));
    tmv.tm_year = Y - 1900;
    tmv.tm_mon = M - 1;
    tmv.tm_mday = D;
    tmv.tm_hour = h;
    tmv.tm_min = mi;
    tmv.tm_sec = sec;
    tmv.tm_isdst = -1;
    return mktime(&tmv);
}
