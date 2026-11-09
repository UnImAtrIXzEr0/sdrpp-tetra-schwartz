// Map export (map_export.h), log time stamps (iso_time.h) and the drive map
// geometry and track (drive_map.h, drive_track.h) on their own:
// writes two small drives (time stamps with UTC offset as in new logs,
// and without as in logs of v0.24.0 and older), exports them and checks the time
// conversion. The KMZ/KML contents are checked by test_tetra_schwartz.py.
#include "map_export.h"
#include "drive_map.h"
#include "map_store.h"
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
    // map tiles: Web Mercator tile coordinates (reference: OSM slippy map formulas), zoom choice, URLs
    {
        double tx, ty, la, lo;
        tiles::toTile(50.1109, 8.6821, 13, tx, ty);
        check(std::fabs(tx - 4293.566009) < 1e-5 && std::fabs(ty - 2774.343445) < 1e-5, "tile coordinates at z13");
        tiles::fromTile(tx, ty, 13, la, lo);
        check(std::fabs(la - 50.1109) < 1e-9 && std::fabs(lo - 8.6821) < 1e-9, "tile coordinates round trip");
        // z13 at 50 N: 12.3 m per tile pixel; z19 is OSM's limit
        check(tiles::zoomFor(50.0, 12.3, 19) == 13 && tiles::zoomFor(50.0, 0.01, 19) == 19 && tiles::zoomFor(50.0, 1e6, 19) == 0, "zoom for the view");
        check(tiles::expand("https://a/{z}/{x}/{y}.png", 13, 4293, 2774) == "https://a/13/4293/2774.png" &&
              tiles::expand("w/{z}/{y}/{x}.jpg", 5, 1, 2) == "w/5/2/1.jpg", "tile URL");
        uint64_t k = tiles::key(19, 274877, 179000);
        check(tiles::keyZ(k) == 19 && tiles::keyX(k) == 274877 && tiles::keyY(k) == 179000, "tile key");
        auto pv = tiles::providers(2026);
        bool attr = !pv.empty();
        for (auto& p : pv) { attr = attr && !p.attribution.empty() && p.url.find("{z}") != std::string::npos; }
        check(attr && pv[1].attribution.find("BKG (2026)") != std::string::npos, "providers with attribution");
    }

    // saved maps: area, tile ranges, names
    {
        mapstore::Area a;
        a.lat = 51.3; a.lon = 9.45; a.rKm = 5;
        double la0, lo0, la1, lo1;
        mapstore::bbox(a, la0, lo0, la1, lo1);
        check(std::fabs(drivemap::distanceM(la0, 9.45, la1, 9.45) - 10000) < 5 && std::fabs(drivemap::distanceM(51.3, lo0, 51.3, lo1) - 10000) < 20, "area 2 x 5 km");
        auto rs = mapstore::ranges(a, 15);
        check(rs.size() == 7 && rs.front().z == 9 && rs.back().z == 15, "zoom levels 9..15");
        // z15 at 51.3 N: tile 763 m wide and high -> 14-15 tiles per 10 km
        check(rs.back().count() >= 14 * 14 && rs.back().count() <= 16 * 16, "tiles at the detail level");
        check(mapstore::countTiles(a, 15) < 400, "tile count");
        a.radius = false; a.wKm = 200; a.hKm = 200;
        check(mapstore::countTiles(a, 17) > mapstore::MAX_TILES, "large area refused at high detail");
        check(mapstore::folderName("Kassel Nord 2026/11") == "Kassel_Nord_202611" && mapstore::folderName("../x") == "x" && mapstore::folderName("G\xC3\xB6ttingen Stra\xC3\x9F" "e") == "Goettingen_Strasse", "folder name");
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
    // map image: rendered without a screen, PNG of the requested size
    {
        CoverageMapExporter e;
        e.fontPath = font;
        CoverageMapExporter::MapImageSpec sp;
        sp.path = std::string(dir) + "/map.png";
        sp.w = 640; sp.h = 480; sp.scale = 1.0f;
        sp.cLat = 51.31; sp.cLon = 9.455; sp.mpp = 5;
        for (int i = 0; i < 100; i++) {
            DrivePoint p;
            p.lat = 51.30 + i * 0.0002; p.lon = 9.45 + i * 0.0001; p.level = -60.0f - i * 0.4f; p.service = i % 10 != 0;
            sp.pts.push_back(p);
        }
        sp.siteValid = true; sp.siteLat = 51.31; sp.siteLon = 9.46;
        sp.title = "test";
        std::string r = e.renderMapImage(sp);
        FILE* f = fopen(sp.path.c_str(), "rb");
        unsigned char hdr[24] = {};
        size_t n = f ? fread(hdr, 1, 24, f) : 0;
        if (f) { fclose(f); }
        int w = (hdr[16] << 24) | (hdr[17] << 16) | (hdr[18] << 8) | hdr[19], h = (hdr[20] << 24) | (hdr[21] << 16) | (hdr[22] << 8) | hdr[23];
        check(r.rfind("Saved ", 0) == 0 && n == 24 && hdr[1] == 'P' && w == 640 && h == 480, "map image 640 x 480 PNG");
    }
    return fails ? 1 : 0;
}
