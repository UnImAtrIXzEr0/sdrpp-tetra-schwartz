// Example coverage map for the manual: synthetic drive around a city centre
// (no real measurement), exported with the program's own map exporter.
#include "map_export.h"
#include <cmath>
#include <cstdio>
#include <random>
int main(int argc, char** argv) {
    const char* csv = argc > 1 ? argv[1] : "drive.csv";
    FILE* f = fopen(csv, "w");
    fprintf(f, "time;level_dbfs;mer_db;evm_rms_pct;evm_peak_pct;carrier_offset_hz;carrier_ppm;ref_freq_hz;symbol_rate_ppm;rx_state;bursts;bursts_tolerated;sync_lost;ber;"
               "crc_bsch_ok;crc_bsch_fail;crc_sb2_ok;crc_sb2_fail;crc_ndb_ok;crc_ndb_fail;crc_schf_ok;crc_schf_fail;mcc;mnc;cc;dl_freq_hz;ul_freq_hz;ts1;ts2;ts3;ts4;"
               "level_dbm;cell_match;gps_fix;lat;lon;alt_m;speed_kmh;sats;hdop\n");
    std::mt19937 g(3);
    std::normal_distribution<double> n(0, 1);
    const double lat0 = 50.1109, lon0 = 8.6821;   // site of the (fictional) repeater
    double lat = 50.105, lon = 8.64, hd = 0.3;
    for (int i = 0; i < 2400; i++) {
        hd += 0.04 * n(g);
        lat += 0.00009 * std::cos(hd);
        lon += 0.00014 * std::sin(hd);
        if (std::fabs(lat - lat0) > 0.045) { hd += 3.14159; }
        if (std::fabs(lon - lon0) > 0.075) { hd += 3.14159; }
        double dkm = std::hypot((lat - lat0) * 111.0, (lon - lon0) * 71.5);
        double dbm = -55.0 - 32.0 * std::log10((std::max)(dkm, 0.2) / 0.2) + 6.0 * n(g) - (std::sin(lon * 900) > 0.8 ? 10 : 0);
        bool svc = dbm > -112;
        double mer = svc ? std::min(30.0, (dbm + 118) * 0.6 + 1.5 * n(g)) : 0;
        int fail = dbm > -100 ? 0 : dbm > -108 ? 1 : 4;
        int h = 12 + i / 3600, m = (i / 60) % 60, s = i % 60;
        fprintf(f, "2026-10-01T%02d:%02d:%02d;%.1f;%.2f;8;20;0;0;439100000;0;%d;68;0;0;%.2e;1;0;2;0;%d;%d;10;%d;901;99;1;439100000;429700000;3;1;1;1;%.1f;1;1;%.6f;%.6f;120;35;9;0.9\n",
                h, m, s, dbm + 40, mer, svc ? 2 : 0, svc ? std::pow(10.0, -(dbm + 130) / 12) : 0.5, 30 - fail, fail, fail, dbm, lat, lon);
    }
    fclose(f);
    CoverageMapExporter e;
    printf("%s\n", e.run(csv, CoverageMapExporter::METRIC_LEVEL, false).c_str());
    e.crcLimit = 1.0f;
    printf("%s\n", e.run(csv, CoverageMapExporter::METRIC_QUALITY, false).c_str());
    return 0;
}
