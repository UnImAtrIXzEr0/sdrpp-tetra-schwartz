#pragma once
// Long-term history of the TETRA measurements (one point per second, 24 h)
// and loading of the CSV logs written by TetraMeasurements, so that a past
// measurement can be shown in the same trend view.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <vector>

struct TetraTrendPoint {
    time_t t = 0;
    float level = NAN;        // dBFS (or dBm when 'calibrated')
    float mer = NAN;          // dB
    float evm = NAN;          // % rms
    float crcFail = NAN;      // % of all CRC checks in this second
    float carrierHz = NAN;    // carrier offset to the VFO
    float bursts = NAN;       // decoded bursts per second
    float syncLost = NAN;     // sync losses in this second
    float tsTraffic[4] = { NAN, NAN, NAN, NAN };   // % of the slot's frames carrying traffic
    bool calibrated = false;  // level in dBm
};

// Ring of trend points; 'seq' counts every point ever added so a reader can
// tell when something changed without copying.
class TetraTrendStore {
public:
    static const size_t CAP = 24 * 3600;

    void add(const TetraTrendPoint& p) {
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
    uint64_t getSeq() {
        std::lock_guard<std::mutex> lck(mtx);
        return seq;
    }
    // copy of the points newer than 'since' (unix time), oldest first
    std::vector<TetraTrendPoint> copy(time_t since = 0) {
        std::lock_guard<std::mutex> lck(mtx);
        std::vector<TetraTrendPoint> out;
        auto it = std::lower_bound(pts.begin(), pts.end(), since, [](const TetraTrendPoint& a, time_t t) { return a.t < t; });
        out.assign(it, pts.end());
        return out;
    }

    // Read a measurement CSV log (tetra_YYYYMMDD_HHMMSS.csv). Columns are
    // found by their header names, so logs of older versions load as well.
    static bool loadCsv(const std::string& path, std::vector<TetraTrendPoint>& out, std::string& err) {
        out.clear();
        std::ifstream f(path);
        if (!f) { err = "Cannot open " + path; return false; }
        std::string line;
        std::map<std::string, int> col;
        while (std::getline(f, line)) {
            if (!line.empty() && line.back() == '\r') { line.pop_back(); }
            if (line.empty() || line[0] == '#') { continue; }
            std::vector<std::string> v = split(line);
            if (col.empty()) {
                for (size_t i = 0; i < v.size(); i++) { col[v[i]] = (int)i; }
                if (!col.count("time") || !col.count("level_dbfs")) { err = "Not a TETRA measurement log (header missing)"; return false; }
                continue;
            }
            auto num = [&](const char* name) -> float {
                auto it = col.find(name);
                if (it == col.end() || it->second >= (int)v.size() || v[it->second].empty()) { return NAN; }
                return (float)atof(v[it->second].c_str());
            };
            TetraTrendPoint p;
            p.t = parseTime(v[col["time"]]);
            if (p.t == 0) { continue; }
            float dbm = num("level_dbm");
            if (!std::isnan(dbm)) { p.level = dbm; p.calibrated = true; }
            else { p.level = num("level_dbfs"); }
            float rxf = num("rx_state");
            bool locked = std::isnan(rxf) || (int)rxf == 2;   // MER only means something with lock
            p.mer = locked ? num("mer_db") : NAN;
            p.evm = locked ? num("evm_rms_pct") : NAN;
            p.carrierHz = num("carrier_offset_hz");
            p.bursts = num("bursts");
            p.syncLost = num("sync_lost");
            double ok = 0, fail = 0;
            const char* oks[4] = { "crc_bsch_ok", "crc_sb2_ok", "crc_ndb_ok", "crc_schf_ok" };
            const char* fails[4] = { "crc_bsch_fail", "crc_sb2_fail", "crc_ndb_fail", "crc_schf_fail" };
            for (int i = 0; i < 4; i++) {
                float a = num(oks[i]), b = num(fails[i]);
                if (!std::isnan(a)) { ok += a; }
                if (!std::isnan(b)) { fail += b; }
            }
            if (ok + fail > 0) { p.crcFail = (float)(100.0 * fail / (ok + fail)); }
            const char* tsn[4] = { "ts1_traffic_pct", "ts2_traffic_pct", "ts3_traffic_pct", "ts4_traffic_pct" };
            for (int i = 0; i < 4; i++) { p.tsTraffic[i] = num(tsn[i]); }
            out.push_back(p);
        }
        if (out.empty()) { err = "No measurement lines in " + path; return false; }
        return true;
    }

private:
    static std::vector<std::string> split(const std::string& s) {
        std::vector<std::string> v;
        size_t a = 0;
        while (true) {
            size_t b = s.find(';', a);
            v.push_back(s.substr(a, b == std::string::npos ? std::string::npos : b - a));
            if (b == std::string::npos) { break; }
            a = b + 1;
        }
        return v;
    }
    // "YYYY-MM-DDTHH:MM:SS" (local time)
    static time_t parseTime(const std::string& s) {
        struct tm tmv;
        memset(&tmv, 0, sizeof(tmv));
        if (sscanf(s.c_str(), "%d-%d-%dT%d:%d:%d", &tmv.tm_year, &tmv.tm_mon, &tmv.tm_mday, &tmv.tm_hour, &tmv.tm_min, &tmv.tm_sec) != 6) { return 0; }
        tmv.tm_year -= 1900;
        tmv.tm_mon -= 1;
        tmv.tm_isdst = -1;
        return mktime(&tmv);
    }

    std::mutex mtx;
    std::deque<TetraTrendPoint> pts;
    uint64_t seq = 0;
};
