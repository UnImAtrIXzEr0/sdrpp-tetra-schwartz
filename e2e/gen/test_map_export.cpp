// Map export (map_export.h) and log time stamps (iso_time.h) on their own:
// writes two small drives (time stamps with UTC offset as in new logs,
// and without as in logs of v0.24.0 and older), exports them and checks the time
// conversion. The KMZ/KML contents are checked by test_tetra_schwartz.py.
#include "map_export.h"
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
