#pragma once
// Drive track: one sample per second with GPS fix, kept in memory for the
// map of the drive test (independent of the CSV log). Written by the
// measurement thread, read by the GUI; 'seq' counts every sample ever added
// so a reader can tell when something changed without copying.

#include <cmath>
#include <cstdint>
#include <ctime>
#include <deque>
#include <mutex>
#include <vector>

struct DrivePoint {
    time_t t = 0;
    double lat = 0, lon = 0;
    float level = NAN;       // dBFS, or dBm when 'dbm'
    float mer = NAN;         // dB
    float ber = NAN;
    float crcPct = NAN;      // % of all CRC checks in this second
    float courseDeg = NAN;   // direction of travel (GPS), NaN unknown
    bool service = false;    // cell received (and matching the lock, if one is set)
    bool dbm = false;
    int mcc = 0, mnc = 0, cc = 0;
    double dlHz = 0;
};

class DriveTrack {
public:
    static const size_t CAP = 24 * 3600;

    void add(const DrivePoint& p) {
        std::lock_guard<std::mutex> lck(mtx);
        pts.push_back(p);
        while (pts.size() > CAP) { pts.pop_front(); }
        seq++;
    }

    void clear() {
        std::lock_guard<std::mutex> lck(mtx);
        pts.clear();
        seq++;
    }

    uint64_t sequence() {
        std::lock_guard<std::mutex> lck(mtx);
        return seq;
    }

    // copies only when the track changed since 'since'; returns the new sequence
    uint64_t copyIfChanged(uint64_t since, std::vector<DrivePoint>& out) {
        std::lock_guard<std::mutex> lck(mtx);
        if (seq != since) { out.assign(pts.begin(), pts.end()); }
        return seq;
    }

private:
    std::mutex mtx;
    std::deque<DrivePoint> pts;
    uint64_t seq = 0;
};
