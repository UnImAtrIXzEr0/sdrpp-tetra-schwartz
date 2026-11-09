// Map export (map_export.h), log time stamps (iso_time.h) and the drive map
// geometry and track (drive_map.h, drive_track.h) on their own:
// writes two small drives (time stamps with UTC offset as in new logs,
// and without as in logs of v0.24.0 and older), exports them and checks the time
// conversion. The KMZ/KML contents are checked by test_tetra_schwartz.py.
#include "map_export.h"
#include "drive_map.h"
#include <cmath>
#include <cstdio>

static int fails = 0;
static void check(bool ok, const char* what) {
    printf("%s %s\n", ok ? "ok:  " : "FAIL:", what);
    if (!ok) { fails++; }
}

static void writeDrive(const char* path, bool withOffset) {
    FILE* f = fopen(path, "w");
    fprintf(f, "# TETRA Analyzer test\n");
    fprintf(f, "time;level_dbfs;mer_db;rx_state;ber;crc_bsch_ok;crc_bsch_fail;mcc;mnc;cc;dl_freq_hz;level_dbm;cell_match;gps_fix;lat;lon\n");
    for (int i = 0; i < 60; i++) {
        // 2026-07-01 12:00:00 local (CEST, +02:00)
        fprintf(f, "2026-07-01T12:00:%02d%s;%.1f;%.1f;%d;%.2e;10;%d;901;9999;1;390012500;%.1f;1;1;%.6f;%.6f\n",
                i, withOffset ? "+02:00" : "", -50.0 - i, 28.0 - i * 0.3, i % 10 == 9 ? 0 : 2, 1e-5 * std::pow(10.0, i / 15.0),
                i % 3, -70.0 - i, 51.3 + i * 0.0004, 9.45 + i * 0.0002);
    }
    fclose(f);
}

int main(int argc, char** argv) {
    // time stamps
    time_t t = parseIsoTime("2026-07-01T12:00:00+02:00");
    check(t == 1782900000, "offset +02:00 parsed as UTC 10:00");
    check(parseIsoTime("2026-07-01T10:00:00Z") == t, "Z parsed");
    check(parseIsoTime("2026-07-01T05:00:00-05:00") == t, "negative offset parsed");
    check(isoUtcTime(t) == "2026-07-01T10:00:00Z", "UTC formatting");
    check(parseIsoTime(isoLocalTime(t)) == t, "local time with offset round trip (summer)");
    check(parseIsoTime(isoLocalTime(1798801200)) == 1798801200, "local time with offset round trip (winter)");
    check(parseIsoTime("2026-07-01T12:00:00") == mktime([] { static struct tm m = {}; m.tm_year = 126; m.tm_mon = 6; m.tm_mday = 1; m.tm_hour = 12; m.tm_isdst = -1; return &m; }()),
          "old log without offset read as local time");
    check(parseIsoTime("no time") == 0, "garbage rejected");

    // drive map: 1-2-5 steps, projection against the great-circle distance
    check(drivemap::niceStep(730) == 500 && drivemap::niceStep(0.31) == 0.2 && drivemap::niceStep(1999) == 1000, "1-2-5 steps");
    DriveMapView mv;
    mv.follow = false;
    {
        double x, y, la, lo;
        mv.project(51.3, 9.45, x, y);   // default view (centre 0/0, 10 m/px): only the round trip matters here
        mv.unproject(x, y, la, lo);
        check(std::fabs(la - 51.3) < 1e-9 && std::fabs(lo - 9.45) < 1e-9, "projection round trip");
    }
    {
        // 5 km east and 5 km north of a point at 51.3 N: plane distance vs. haversine
        double d1 = drivemap::distanceM(51.3, 9.45, 51.3, 9.45 + 5000 / (drivemap::M_PER_DEG * std::cos(51.3 * drivemap::PI / 180)));
        double d2 = drivemap::distanceM(51.3, 9.45, 51.3 + 5000 / drivemap::M_PER_DEG, 9.45);
        check(std::fabs(d1 - 5000) < 2.0 && std::fabs(d2 - 5000) < 0.5, "local projection within 2 m on 5 km");
    }
    // drive track: ring capacity and change detection
    DriveTrack tr;
    std::vector<DrivePoint> cp;
    uint64_t sq = tr.copyIfChanged((uint64_t)-1, cp);
    DrivePoint dp;
    for (size_t i = 0; i < DriveTrack::CAP + 10; i++) { dp.t = (time_t)i; tr.add(dp); }
    uint64_t sq2 = tr.copyIfChanged(sq, cp);
    check(cp.size() == DriveTrack::CAP && cp.front().t == 10 && cp.back().t == (time_t)(DriveTrack::CAP + 9), "track keeps the newest 24 h");
    cp.clear();
    check(tr.copyIfChanged(sq2, cp) == sq2 && cp.empty(), "no copy without a change");

    const char* dir = argc > 1 ? argv[1] : ".";
    std::string font = argc > 2 ? argv[2] : "";
    for (int k = 0; k < 2; k++) {
        std::string csv = std::string(dir) + (k == 0 ? "/drive_new.csv" : "/drive_old.csv");
        writeDrive(csv.c_str(), k == 0);
        CoverageMapExporter e;
        e.fontPath = font;
        std::string r = e.run(csv, CoverageMapExporter::METRIC_MER, false);
        printf("%s\n", r.c_str());
        check(r.rfind("Exported 60 points (6 without service)", 0) == 0, k == 0 ? "export (new log)" : "export (old log)");
    }
    return fails ? 1 : 0;
}
