#pragma once
// Coverage map export from a measurement CSV:
//  - KMZ and KML for Google Earth: one folder per value (level, MER, BER,
//    quality), points without service, the route as a line, time stamps
//    for the time slider and all values in the balloon; the KMZ also holds
//    the point symbol and a legend per value, so it works offline
//  - self-contained HTML with OpenStreetMap (Leaflet), colored points,
//    optional heatmap layer and legend
// Runs in a background thread; never touches the DSP chain.
// Contains the stb_image_write and stb_truetype implementations (static),
// so include it in one translation unit only.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "coverage_scale.h"
#include "drive_track.h"
#include "iso_time.h"
#include "map_tiles.h"

#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <imgui/stb_image_write.h>
#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#include <imgui/imstb_truetype.h>

class CoverageMapExporter {
public:
    enum Metric { METRIC_LEVEL = 0, METRIC_MER = 1, METRIC_BER = 2, METRIC_QUALITY = 3 };
    float crcLimit = 1.0f;   // METRIC_QUALITY: good up to this, marginal up to 10x
    std::string fontPath;    // TTF for the legend texts in the KMZ (empty: bar only)

    ~CoverageMapExporter() {
        if (worker.joinable()) { worker.join(); }
    }

    bool busy() { return isBusy.load(); }

    std::string status() {
        std::lock_guard<std::mutex> lck(mtx);
        return statusText;
    }

    void exportAsync(const std::string& csvPath, Metric metric, bool onlyLockedCell) {
        if (isBusy.exchange(true)) { return; }
        if (worker.joinable()) { worker.join(); }
        worker = std::thread([this, csvPath, metric, onlyLockedCell]() {
            std::string res = run(csvPath, metric, onlyLockedCell);
            setStatus(res);
            isBusy = false;
        });
    }

    // One sample with position from a measurement CSV
    struct Row {
        double lat = 0, lon = 0;
        time_t t = 0;          // 0: no valid time stamp
        std::string time;      // as in the CSV
        double level = 0, mer = 0, ber = 0, crcPct = 0, dlHz = 0;
        std::string berS, mcc, mnc, cc;
        bool service = false;  // cell received (and matching the lock, if requested)
    };

    // Reads the samples with GPS fix of a measurement CSV; returns an error text or ""
    static std::string readCsv(const std::string& csvPath, bool onlyLockedCell, std::vector<Row>& rows, bool& useDbm) {
        std::ifstream in(csvPath);
        if (!in) { return "Cannot read " + csvPath; }

        std::string line;
        std::vector<std::string> header;
        while (std::getline(in, line)) {
            if (!line.empty() && line[0] == '#') { continue; }
            header = split(line);
            break;
        }
        auto col = [&](const char* n) {
            for (size_t i = 0; i < header.size(); i++) { if (header[i] == n) { return (int)i; } }
            return -1;
        };
        int cTime = col("time"), cLat = col("lat"), cLon = col("lon"), cFix = col("gps_fix");
        int cDbm = col("level_dbm"), cDbfs = col("level_dbfs"), cMer = col("mer_db"), cBer = col("ber");
        int cState = col("rx_state"), cMatch = col("cell_match"), cMcc = col("mcc"), cMnc = col("mnc"), cCc = col("cc");
        int cDl = col("dl_freq_hz");
        int cCrcOk[4] = { col("crc_bsch_ok"), col("crc_sb2_ok"), col("crc_ndb_ok"), col("crc_schf_ok") };
        int cCrcFail[4] = { col("crc_bsch_fail"), col("crc_sb2_fail"), col("crc_ndb_fail"), col("crc_schf_fail") };
        if (cLat < 0 || cLon < 0) { return "CSV has no position columns (log from v0.5.0 or newer needed)"; }

        rows.clear();
        useDbm = false;
        while (std::getline(in, line)) {
            auto f = split(line);
            auto get = [&](int c) { return (c >= 0 && c < (int)f.size()) ? f[c] : std::string(); };
            if (cFix >= 0 && get(cFix) != "1") { continue; }
            std::string sLat = get(cLat), sLon = get(cLon);
            if (sLat.empty() || sLon.empty()) { continue; }
            Row r;
            r.lat = atof(sLat.c_str());
            r.lon = atof(sLon.c_str());
            if (r.lat == 0.0 && r.lon == 0.0) { continue; }
            int state = atoi(get(cState).c_str());
            std::string match = get(cMatch);
            r.service = (state == 2) && !(onlyLockedCell && match == "0");
            std::string dbm = get(cDbm);
            if (!dbm.empty()) { r.level = atof(dbm.c_str()); useDbm = true; }
            else { r.level = atof(get(cDbfs).c_str()); }
            r.mer = atof(get(cMer).c_str());
            r.berS = get(cBer);
            r.ber = atof(r.berS.c_str());
            double ok = 0, fail = 0;
            for (int i = 0; i < 4; i++) { ok += atof(get(cCrcOk[i]).c_str()); fail += atof(get(cCrcFail[i]).c_str()); }
            r.crcPct = ok + fail > 0 ? 100.0 * fail / (ok + fail) : NAN;   // no CRC check in this second: no data
            r.time = get(cTime);
            r.t = parseIsoTime(r.time);
            r.mcc = get(cMcc);
            r.mnc = get(cMnc);
            r.cc = get(cCc);
            r.dlHz = atof(get(cDl).c_str());
            rows.push_back(r);
        }
        if (rows.empty()) { return "No samples with GPS position in " + csvPath; }
        return "";
    }

    // Synchronous core (public for tests)
    std::string run(const std::string& csvPath, Metric metric, bool onlyLockedCell) {
        std::vector<Row> rows;
        bool useDbm = false;
        std::string err = readCsv(csvPath, onlyLockedCell, rows, useDbm);
        if (!err.empty()) { return err; }

        // Downsample very long drives so Google Earth / browsers stay responsive
        const size_t MAX_PTS = 20000;
        if (rows.size() > MAX_PTS) {
            std::vector<Row> d;
            double step = (double)rows.size() / MAX_PTS;
            for (double i = 0; i < rows.size(); i += step) { d.push_back(rows[(size_t)i]); }
            rows.swap(d);
        }

        Scale sc[4];
        for (int m = 0; m < 4; m++) { sc[m] = scaleFor((Metric)m, useDbm); }

        // HTML: the selected value only (as before)
        std::vector<Pt> pts;
        for (auto& r : rows) {
            Pt p;
            p.lat = r.lat;
            p.lon = r.lon;
            p.v = value(r, metric);
            p.service = r.service;
            if (p.service && std::isnan(p.v)) { continue; }
            char lb[256], crc[16];
            if (std::isnan(r.crcPct)) { snprintf(crc, sizeof(crc), "-"); } else { snprintf(crc, sizeof(crc), "%.1f %%", r.crcPct); }
            snprintf(lb, sizeof(lb), "%s | Level %.1f %s | MER %.1f dB | BER %s | CRC errors %s | MCC %s MNC %s CC %s%s",
                     r.time.c_str(), r.level, useDbm ? "dBm" : "dBFS", r.mer, r.berS.c_str(), crc,
                     r.mcc.c_str(), r.mnc.c_str(), r.cc.c_str(), r.service ? "" : " | NO SERVICE");
            p.label = lb;
            pts.push_back(p);
        }
        const Scale& s = sc[metric];
        auto norm = [&](double v) { return s.norm(v); };

        std::string base = csvPath.size() > 4 ? csvPath.substr(0, csvPath.size() - 4) : csvPath;
        std::string suffix = (metric == METRIC_LEVEL ? "_level" : metric == METRIC_MER ? "_mer" : metric == METRIC_QUALITY ? "_quality" : "_ber");
        std::string kmlPath = base + ".kml";
        std::string kmzPath = base + ".kmz";
        std::string htmlPath = base + suffix + "_map.html";
        std::string docName = documentName(rows);
        std::string fileName = csvPath.substr(csvPath.find_last_of("/\\") == std::string::npos ? 0 : csvPath.find_last_of("/\\") + 1);

        if (!writeText(kmlPath, buildKml(rows, sc, metric, false, docName, fileName))) { return "Cannot write " + kmlPath; }
        std::vector<ZipEntry> z;
        z.push_back({ "doc.kml", buildKml(rows, sc, metric, true, docName, fileName), true });
        z.push_back({ "files/dot.png", dotPng(), false });
        for (int m = 0; m < 4; m++) { z.push_back({ "files/" + legendName((Metric)m), legendPng(sc[m]), false }); }
        if (!writeZip(kmzPath, z)) { return "Cannot write " + kmzPath; }
        if (!writeHtml(htmlPath, pts, norm, s.name, s.unit, s.lo, s.hi, s.logScale)) { return "Cannot write " + htmlPath; }
        size_t noSvc = std::count_if(rows.begin(), rows.end(), [](const Row& r) { return !r.service; });
        return "Exported " + std::to_string(rows.size()) + " points (" + std::to_string(noSvc) + " without service): " +
               kmzPath + ", " + kmlPath + " and " + htmlPath;
    }

    // ---- map image (PNG) of the drive map ----
    struct MapImageSpec {
        std::string path;
        int w = 1600, h = 1200;          // output pixels
        double cLat = 0, cLon = 0, mpp = 10;   // view centre, metres per output pixel
        std::vector<DrivePoint> pts;
        int metric = 0;
        bool siteValid = false;
        double siteLat = 0, siteLon = 0;
        int tileZ = -1;                  // -1: no map
        std::function<std::string(int, int, int)> tileFile;   // z, x, y -> file ("" none)
        std::string attribution, title;
        float tileAlpha = 0.75f;
        int bgR = 200, bgG = 205, bgB = 210;
        float scale = 2.0f;              // size of symbols and texts against the screen
    };

    void exportImageAsync(const MapImageSpec& spec) {
        if (isBusy.exchange(true)) { return; }
        if (worker.joinable()) { worker.join(); }
        worker = std::thread([this, spec]() {
            setStatus(renderMapImage(spec));
            isBusy = false;
        });
    }

    // Software rendering (independent of the screen): tiles from the cache or the
    // saved map, grid without a map, cell site with rings, points, legend, scale
    // bar, north arrow, title and the attribution of the map
    std::string renderMapImage(const MapImageSpec& sp) {
        const double M = 6371008.8 * tiles::PI / 180.0;
        const double kx = M * std::cos(sp.cLat * tiles::PI / 180), ky = M;
        int W = sp.w, H = sp.h;
        Canvas cv(W, H);
        cv.rect(0, 0, W, H, sp.bgR, sp.bgG, sp.bgB, 1.0);
        auto toPx = [&](double lat, double lon, double& x, double& y) {
            x = W * 0.5 + (lon - sp.cLon) * kx / sp.mpp;
            y = H * 0.5 - (lat - sp.cLat) * ky / sp.mpp;
        };
        float k = sp.scale;
        std::vector<unsigned char> ttf;
        stbtt_fontinfo fi;
        const stbtt_fontinfo* font = loadFont(fontPath, ttf, fi);
        auto textW = [&](float px, const std::string& t) { Canvas d(1, 1); return drawText(d, font, px, 0, 0, t, true); };

        if (sp.tileZ >= 0 && sp.tileFile) {
            // tile coordinates per column (longitude) and per row (latitude)
            int z = sp.tileZ;
            std::vector<double> tx(W), ty(H);
            for (int x = 0; x < W; x++) {
                double lon = sp.cLon + (x + 0.5 - W * 0.5) * sp.mpp / kx, a, b;
                tiles::toTile(sp.cLat, lon, z, a, b);
                tx[x] = a;
            }
            for (int y = 0; y < H; y++) {
                double lat = sp.cLat - (y + 0.5 - H * 0.5) * sp.mpp / ky, a, b;
                tiles::toTile(lat, sp.cLon, z, a, b);
                ty[y] = b;
            }
            struct Img { int w = 0, h = 0; std::vector<uint8_t> px; };
            std::map<uint64_t, Img> imgs;
            auto get = [&](int x, int y) -> const Img* {
                uint64_t key = tiles::key(z, x, y);
                auto it = imgs.find(key);
                if (it == imgs.end()) {
                    Img im;
                    std::string f = sp.tileFile(z, x, y);
                    int w, h, n;
                    stbi_uc* d = f.empty() ? nullptr : stbi_load(f.c_str(), &w, &h, &n, 4);
                    if (d) { im.w = w; im.h = h; im.px.assign(d, d + (size_t)w * h * 4); stbi_image_free(d); }
                    it = imgs.emplace(key, std::move(im)).first;
                }
                return it->second.w ? &it->second : nullptr;
            };
            int n = 1 << z;
            for (int y = 0; y < H; y++) {
                for (int x = 0; x < W; x++) {
                    double fx = tx[x], fy = ty[y];
                    if (fx < 0 || fy < 0 || fx >= n || fy >= n) { continue; }
                    int ix = (int)fx, iy = (int)fy;
                    const Img* im = get(ix, iy);
                    if (!im) { continue; }
                    // bilinear sample inside the tile
                    double u = (fx - ix) * im->w - 0.5, v = (fy - iy) * im->h - 0.5;
                    int u0 = std::clamp((int)std::floor(u), 0, im->w - 1), v0 = std::clamp((int)std::floor(v), 0, im->h - 1);
                    int u1 = (std::min)(u0 + 1, im->w - 1), v1 = (std::min)(v0 + 1, im->h - 1);
                    double au = std::clamp(u - u0, 0.0, 1.0), av = std::clamp(v - v0, 0.0, 1.0);
                    double c[3];
                    for (int ch = 0; ch < 3; ch++) {
                        double p00 = im->px[(v0 * im->w + u0) * 4 + ch], p01 = im->px[(v0 * im->w + u1) * 4 + ch];
                        double p10 = im->px[(v1 * im->w + u0) * 4 + ch], p11 = im->px[(v1 * im->w + u1) * 4 + ch];
                        c[ch] = (p00 * (1 - au) + p01 * au) * (1 - av) + (p10 * (1 - au) + p11 * au) * av;
                    }
                    cv.blend(x, y, (int)c[0], (int)c[1], (int)c[2], sp.tileAlpha);
                }
            }
        }
        else {
            // grid in 1-2-5 metre steps, fixed to the ground
            double step = drivemapNice(sp.mpp * 120.0 * k);
            double x0 = sp.cLon * kx - W * 0.5 * sp.mpp, x1 = sp.cLon * kx + W * 0.5 * sp.mpp;
            for (double g = std::ceil(x0 / step) * step; g <= x1; g += step) {
                int x = (int)std::lround(W * 0.5 + (g - sp.cLon * kx) / sp.mpp);
                cv.rect(x, 0, x + 1, H, 110, 115, 120, 0.35);
            }
            double y0 = sp.cLat * ky - H * 0.5 * sp.mpp, y1 = sp.cLat * ky + H * 0.5 * sp.mpp;
            for (double g = std::ceil(y0 / step) * step; g <= y1; g += step) {
                int y = (int)std::lround(H * 0.5 - (g - sp.cLat * ky) / sp.mpp);
                cv.rect(0, y, W, y + 1, 110, 115, 120, 0.35);
            }
        }

        // cell site: distance rings under the track
        double sx = 0, sy = 0;
        if (sp.siteValid) {
            toPx(sp.siteLat, sp.siteLon, sx, sy);
            double farM = 0;
            for (int c = 0; c < 4; c++) { farM = (std::max)(farM, std::hypot((c & 1 ? W : 0) - sx, (c & 2 ? H : 0) - sy) * sp.mpp); }
            double step = drivemapNice((std::min)(W, H) * 0.5 * sp.mpp / 2.5);
            int n = 0;
            for (double r = step; r <= farM && n < 40; r += step, n++) {
                double rp = r / sp.mpp;
                cv.ring(sx, sy, rp, 1.2 * k, 0, 64, 140, 0.7);
                char t[32];
                if (r >= 1000) { snprintf(t, sizeof(t), "%g km", r / 1000); } else { snprintf(t, sizeof(t), "%g m", r); }
                drawText(cv, font, 13 * k, (int)(sx + 3 * k), (int)(sy - rp - 3 * k), t, false, 0, 64, 140);
            }
        }

        // points: like on the screen, one per cell of 3 screen pixels (newest wins), drawn oldest first
        cov::Scale sc = cov::scaleFor(sp.metric, !sp.pts.empty() && sp.pts.back().dbm, crcLimit);
        double cell = 3.0 * k;
        int gw = (int)(W / cell) + 1, gh = (int)(H / cell) + 1;
        std::vector<unsigned char> occ((size_t)gw * gh, 0);
        std::vector<size_t> sel;
        for (size_t i = sp.pts.size(); i-- > 0;) {
            double x, y;
            toPx(sp.pts[i].lat, sp.pts[i].lon, x, y);
            if (x < -10 || y < -10 || x > W + 10 || y > H + 10) { continue; }
            int gx = std::clamp((int)(x / cell), 0, gw - 1), gy = std::clamp((int)(y / cell), 0, gh - 1);
            if (occ[(size_t)gy * gw + gx]) { continue; }
            occ[(size_t)gy * gw + gx] = 1;
            sel.push_back(i);
        }
        for (size_t si = sel.size(); si-- > 0;) {
            const DrivePoint& p = sp.pts[sel[si]];
            double x, y;
            toPx(p.lat, p.lon, x, y);
            if (!p.service) { cv.disc(x, y, 3.5 * k, 0, 0, 0, 200, 200, 200, 0.5 * k); continue; }
            double v;
            switch (sp.metric) {
            case cov::LEVEL: v = p.level; break;
            case cov::MER: v = p.mer; break;
            case cov::BER: v = (std::isnan(p.ber) || p.ber < 0) ? NAN : p.ber; break;
            default: v = std::isnan(p.crcPct) ? NAN : cov::quality(p.crcPct, crcLimit); break;
            }
            if (std::isnan(v)) { continue; }
            int r, g, b;
            cov::ramp(sc.norm(v), r, g, b);
            cv.disc(x, y, 3.5 * k, r, g, b, 20, 20, 20, 0.5 * k);
        }

        // cell site symbol and label on top
        if (sp.siteValid) {
            double hh = 14 * k;
            for (int y = (int)(sy - hh); y <= (int)(sy + hh * 0.35); y++) {
                double t = (y - (sy - hh)) / (hh * 1.35);
                double half = t * hh * 0.45;
                for (int x = (int)(sx - half - 2 * k); x <= (int)(sx + half + 2 * k); x++) {
                    double e = std::fabs(std::fabs(x + 0.5 - sx) - half);
                    if (e < 1.2 * k || y >= (int)(sy + hh * 0.35) - (int)(1.5 * k)) { cv.blend(x, y, 0, 64, 140, 1.0); }
                }
            }
            cv.disc(sx, sy - hh, 3 * k, 0, 64, 140, 0, 64, 140, 0);
            std::string lbl = "Cell site";
            int tw = textW(14 * k, lbl);
            int lx = (int)(sx + hh * 0.7), ly = (int)(sy - hh * 0.3);
            cv.rect(lx - (int)(2 * k), ly - (int)(2 * k), lx + tw + (int)(2 * k), ly + (int)(16 * k), 240, 242, 245, 0.8);
            drawText(cv, font, 14 * k, lx, ly + (int)(12 * k), lbl, false, 0, 64, 140);
        }

        // title strip
        if (font && !sp.title.empty()) {
            cv.rect(0, 0, W, (int)(26 * k), 24, 26, 28, 0.8);
            drawText(cv, font, 16 * k, (int)(10 * k), (int)(18 * k), sp.title);
        }
        // north arrow (top right)
        {
            double x = W - 22 * k, y = (sp.title.empty() ? 10 : 34) * k;
            cv.rect((int)(x - 11 * k), (int)(y - 4 * k), (int)(x + 11 * k), (int)(y + 40 * k), 240, 242, 245, 0.7);
            for (int yy = 0; yy < (int)(18 * k); yy++) {
                double half = 7 * k * yy / (18 * k);
                cv.rect((int)(x - half), (int)(y + yy), (int)(x + half + 1), (int)(y + yy + 1), 20, 20, 20, 1.0);
            }
            if (font) { drawText(cv, font, 14 * k, (int)(x - textW(14 * k, "N") * 0.5), (int)(y + 34 * k), "N", false, 20, 20, 20); }
        }
        // scale bar (bottom right) and attribution above it
        {
            double len = drivemapNice(sp.mpp * 140.0 * k);
            int px = (int)(len / sp.mpp), x1 = W - (int)(14 * k), x0 = x1 - px, y = H - (int)(14 * k);
            char t[32];
            if (len >= 1000) { snprintf(t, sizeof(t), "%g km", len / 1000); } else { snprintf(t, sizeof(t), "%g m", len); }
            cv.rect(x0 - (int)(6 * k), y - (int)(26 * k), x1 + (int)(6 * k), y + (int)(5 * k), 240, 242, 245, 0.7);
            cv.rect(x0, y - (int)(2 * k), x1, y, 20, 20, 20, 1.0);
            cv.rect(x0, y - (int)(7 * k), x0 + (int)(2 * k), y, 20, 20, 20, 1.0);
            cv.rect(x1 - (int)(2 * k), y - (int)(7 * k), x1, y, 20, 20, 20, 1.0);
            if (font) { drawText(cv, font, 14 * k, (x0 + x1) / 2 - textW(14 * k, t) / 2, y - (int)(9 * k), t, false, 20, 20, 20); }
            if (font && !sp.attribution.empty()) {
                int aw = textW(12 * k, sp.attribution);
                int ax = (std::max)((int)(4 * k), W - aw - (int)(14 * k)), ay = y - (int)(34 * k);
                cv.rect(ax - (int)(4 * k), ay - (int)(13 * k), ax + aw + (int)(4 * k), ay + (int)(5 * k), 240, 242, 245, 0.8);
                drawText(cv, font, 12 * k, ax, ay, sp.attribution, false, 20, 20, 20);
            }
        }
        // legend (bottom left), at the scale of the image
        {
            Canvas big = legendCanvasScaled(sc, font, k);
            cv.blit(big, (int)(10 * k), H - big.h - (int)(10 * k));
        }
        std::string png = encodePng(cv.px, W, H);
        if (png.empty() || !writeText(sp.path, png)) { return "Cannot write " + sp.path; }
        return "Saved " + sp.path;
    }

private:

    struct Pt {
        double lat, lon;
        double v;      // metric value
        bool service;
        std::string label;
    };

    // Fixed scales so maps of different drives are comparable (coverage_scale.h)
    using Scale = cov::Scale;
    Scale scaleFor(Metric m, bool useDbm) const { return cov::scaleFor(m, useDbm, crcLimit); }

    double value(const Row& r, Metric m) const {
        switch (m) {
        case METRIC_LEVEL: return r.level;
        case METRIC_MER: return r.mer;
        case METRIC_BER: return (r.berS.empty() || r.ber < 0) ? NAN : r.ber;   // -1: not measured
        default: return std::isnan(r.crcPct) ? NAN : cov::quality(r.crcPct, crcLimit);
        }
    }

    static const char* metricKey(Metric m) {
        return m == METRIC_LEVEL ? "level" : m == METRIC_MER ? "mer" : m == METRIC_BER ? "ber" : "quality";
    }

    static std::string legendName(Metric m) { return std::string("legend_") + metricKey(m) + ".png"; }

    // "TETRA coverage - MCC 901 MNC 9999 CC 1, 390.0125 MHz, 2026-11-09"
    static std::string documentName(const std::vector<Row>& rows) {
        std::map<std::string, int> cells;
        std::map<double, int> freqs;
        for (auto& r : rows) {
            if (!r.service || r.mcc.empty()) { continue; }
            cells["MCC " + r.mcc + " MNC " + r.mnc + " CC " + r.cc]++;
            if (r.dlHz > 0) { freqs[r.dlHz]++; }
        }
        std::string n = "TETRA coverage";
        auto most = [](auto& mp) { return std::max_element(mp.begin(), mp.end(), [](auto& a, auto& b) { return a.second < b.second; }); };
        if (!cells.empty()) { n += " - " + most(cells)->first; }
        if (!freqs.empty()) {
            char b[32];
            snprintf(b, sizeof(b), "%.4f MHz", most(freqs)->first / 1e6);
            n += (cells.empty() ? " - " : ", ") + std::string(b);
        }
        if (rows.front().time.size() >= 10) { n += ", " + rows.front().time.substr(0, 10); }
        return n;
    }

    std::string buildKml(const std::vector<Row>& rows, const Scale* sc, Metric selected, bool kmz,
                         const std::string& docName, const std::string& fileName) const {
        std::string o;
        o.reserve(rows.size() * 1200 + 4096);
        char b[512];
        const char* icon = kmz ? "files/dot.png" : "http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png";
        o += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n";
        o += "<name>" + xmlEscape(docName) + "</name>\n";
        o += "<description>SDR++ Air Interface Analyzer, " + xmlEscape(fileName) + ", " + std::to_string(rows.size()) +
             " samples. Black: no service.</description>\n";
        const int BINS = 10;
        for (int i = 0; i <= BINS; i++) {
            int r, g, bl;
            ramp((double)i / BINS, r, g, bl);
            // KML colors are aabbggrr
            snprintf(b, sizeof(b), "<Style id=\"s%d\"><IconStyle><color>ff%02x%02x%02x</color><scale>0.6</scale>"
                                   "<Icon><href>%s</href></Icon></IconStyle><LabelStyle><scale>0</scale></LabelStyle></Style>\n",
                     i, bl, g, r, icon);
            o += b;
        }
        snprintf(b, sizeof(b), "<Style id=\"nosvc\"><IconStyle><color>ff000000</color><scale>0.6</scale>"
                               "<Icon><href>%s</href></Icon></IconStyle><LabelStyle><scale>0</scale></LabelStyle></Style>\n", icon);
        o += b;
        o += "<Style id=\"route\"><LineStyle><color>c0ffffff</color><width>2</width></LineStyle></Style>\n";

        auto placemark = [&](const Row& r, const std::string& style) {
            o += "<Placemark>";
            if (r.t != 0) { o += "<TimeStamp><when>" + isoUtcTime(r.t) + "</when></TimeStamp>"; }
            o += "<styleUrl>#" + style + "</styleUrl><ExtendedData>";
            auto data = [&](const char* n, const std::string& v) {
                o += "<Data name=\"";
                o += n;
                o += "\"><value>" + xmlEscape(v) + "</value></Data>";
            };
            data("Time", r.time);
            snprintf(b, sizeof(b), "%.1f %s", r.level, sc[METRIC_LEVEL].unit.c_str());
            data("Level", b);
            snprintf(b, sizeof(b), "%.1f dB", r.mer);
            data("MER", b);
            data("BER", r.berS);
            if (std::isnan(r.crcPct)) { snprintf(b, sizeof(b), "-"); } else { snprintf(b, sizeof(b), "%.1f %%", r.crcPct); }
            data("CRC errors", b);
            if (!r.mcc.empty()) { data("Cell", "MCC " + r.mcc + " MNC " + r.mnc + " CC " + r.cc); }
            if (r.dlHz > 0) {
                snprintf(b, sizeof(b), "%.4f MHz", r.dlHz / 1e6);
                data("Downlink", b);
            }
            data("Service", r.service ? "yes" : "no");
            snprintf(b, sizeof(b), "</ExtendedData><Point><coordinates>%.6f,%.6f,0</coordinates></Point></Placemark>\n", r.lon, r.lat);
            o += b;
        };

        for (int m = 0; m < 4; m++) {
            const Scale& s = sc[m];
            o += "<Folder><name>" + xmlEscape(s.title) + "</name>";
            o += std::string("<visibility>") + (m == selected ? "1" : "0") + "</visibility>";
            o += "<description>Red = " + xmlEscape(s.loText + (s.unit.empty() || m == METRIC_QUALITY ? "" : " " + s.unit)) +
                 ", green = " + xmlEscape(s.hiText + (s.unit.empty() || m == METRIC_QUALITY ? "" : " " + s.unit)) + "</description>\n";
            if (kmz) {
                o += "<ScreenOverlay><name>Legend</name><Icon><href>files/" + legendName((Metric)m) + "</href></Icon>"
                     "<overlayXY x=\"0\" y=\"0\" xunits=\"fraction\" yunits=\"fraction\"/>"
                     "<screenXY x=\"10\" y=\"30\" xunits=\"pixels\" yunits=\"pixels\"/>"
                     "<size x=\"0\" y=\"0\" xunits=\"pixels\" yunits=\"pixels\"/></ScreenOverlay>\n";
            }
            for (auto& r : rows) {
                if (!r.service) { continue; }
                double v = value(r, (Metric)m);
                if (std::isnan(v)) { continue; }
                placemark(r, "s" + std::to_string((int)std::lround(s.norm(v) * BINS)));
            }
            o += "</Folder>\n";
        }
        o += "<Folder><name>No service</name>\n";
        for (auto& r : rows) {
            if (r.service) { continue; }
            placemark(r, "nosvc");
        }
        o += "</Folder>\n";
        o += "<Folder><name>Route</name><Placemark><name>Route</name><styleUrl>#route</styleUrl><LineString><tessellate>1</tessellate><coordinates>\n";
        for (auto& r : rows) {
            snprintf(b, sizeof(b), "%.6f,%.6f,0\n", r.lon, r.lat);
            o += b;
        }
        o += "</coordinates></LineString></Placemark></Folder>\n";
        o += "</Document>\n</kml>\n";
        return o;
    }

    // ---- images for the KMZ ----
    static void pngWriter(void* ctx, void* data, int size) {
        auto* v = (std::string*)ctx;
        v->append((const char*)data, size);
    }

    static std::string encodePng(const std::vector<uint8_t>& rgba, int w, int h) {
        std::string out;
        stbi_write_png_to_func(pngWriter, &out, w, h, 4, rgba.data(), w * 4);
        return out;
    }

    // White disk with a dark rim; Google Earth multiplies it with the style color
    static std::string dotPng() {
        const int N = 32;
        std::vector<uint8_t> img(N * N * 4, 0);
        double c = (N - 1) / 2.0, rOut = N / 2.0 - 1.0, rim = 2.5;
        for (int y = 0; y < N; y++) {
            for (int x = 0; x < N; x++) {
                double d = std::sqrt((x - c) * (x - c) + (y - c) * (y - c));
                double a = (std::min)(1.0, (std::max)(0.0, rOut - d + 0.5));
                uint8_t v = d > rOut - rim ? 60 : 255;
                uint8_t* p = &img[(y * N + x) * 4];
                p[0] = p[1] = p[2] = v;
                p[3] = (uint8_t)std::lround(a * 255);
            }
        }
        return encodePng(img, N, N);
    }

    struct Canvas {
        int w, h;
        std::vector<uint8_t> px;
        Canvas(int w_, int h_) : w(w_), h(h_), px(w_ * h_ * 4, 0) {}
        void blend(int x, int y, int r, int g, int b, double a) {
            if (x < 0 || y < 0 || x >= w || y >= h || a <= 0) { return; }
            uint8_t* p = &px[(y * w + x) * 4];
            double da = p[3] / 255.0, oa = a + da * (1 - a);
            if (oa <= 0) { return; }
            p[0] = (uint8_t)std::lround((r * a + p[0] * da * (1 - a)) / oa);
            p[1] = (uint8_t)std::lround((g * a + p[1] * da * (1 - a)) / oa);
            p[2] = (uint8_t)std::lround((b * a + p[2] * da * (1 - a)) / oa);
            p[3] = (uint8_t)std::lround(oa * 255);
        }
        void rect(int x0, int y0, int x1, int y1, int r, int g, int b, double a) {
            for (int y = y0; y < y1; y++) { for (int x = x0; x < x1; x++) { blend(x, y, r, g, b, a); } }
        }
        // filled circle with a rim (anti-aliased edges)
        void disc(double cx, double cy, double rad, int r, int g, int b, int rr, int rg, int rb, double rimW) {
            for (int y = (int)std::floor(cy - rad - 1); y <= (int)std::ceil(cy + rad + 1); y++) {
                for (int x = (int)std::floor(cx - rad - 1); x <= (int)std::ceil(cx + rad + 1); x++) {
                    double d = std::hypot(x + 0.5 - cx, y + 0.5 - cy);
                    double a = std::clamp(rad - d + 0.5, 0.0, 1.0);
                    if (a <= 0) { continue; }
                    bool rim = d > rad - rimW;
                    blend(x, y, rim ? rr : r, rim ? rg : g, rim ? rb : b, a);
                }
            }
        }
        // circle outline of width w
        void ring(double cx, double cy, double rad, double w, int r, int g, int b, double a) {
            int x0 = (std::max)(0, (int)std::floor(cx - rad - w)), x1 = (std::min)(this->w - 1, (int)std::ceil(cx + rad + w));
            int y0 = (std::max)(0, (int)std::floor(cy - rad - w)), y1 = (std::min)(h - 1, (int)std::ceil(cy + rad + w));
            for (int y = y0; y <= y1; y++) {
                double dy = y + 0.5 - cy;
                if (std::fabs(dy) > rad + w) { continue; }
                for (int x = x0; x <= x1; x++) {
                    double d = std::fabs(std::hypot(x + 0.5 - cx, dy) - rad);
                    double c = std::clamp(w * 0.5 - d + 0.5, 0.0, 1.0);
                    if (c > 0) { blend(x, y, r, g, b, a * c); }
                }
            }
        }
        void blit(const Canvas& src, int dx, int dy) {
            for (int y = 0; y < src.h; y++) {
                for (int x = 0; x < src.w; x++) {
                    const uint8_t* p = &src.px[(y * src.w + x) * 4];
                    if (p[3]) { blend(dx + x, dy + y, p[0], p[1], p[2], p[3] / 255.0); }
                }
            }
        }
    };

    static const stbtt_fontinfo* loadFont(const std::string& path, std::vector<unsigned char>& ttf, stbtt_fontinfo& fi) {
        if (path.empty()) { return nullptr; }
        std::ifstream f(path, std::ios::binary);
        if (!f) { return nullptr; }
        ttf.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        if (ttf.empty() || !stbtt_InitFont(&fi, ttf.data(), stbtt_GetFontOffsetForIndex(ttf.data(), 0))) { return nullptr; }
        return &fi;
    }

    // Draws UTF-8 text with its baseline at y; returns the advance in pixels (0 without font)
    static int drawText(Canvas& cv, const stbtt_fontinfo* font, float px, int x, int y, const std::string& text, bool measureOnly = false,
                        int cr = 240, int cg = 240, int cb = 240) {
        if (!font) { return 0; }
        float scale = stbtt_ScaleForPixelHeight(font, px);
        float pen = (float)x;
        for (size_t i = 0; i < text.size();) {
            unsigned c = (unsigned char)text[i];
            int len = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : 4;
            unsigned cp = len == 1 ? c : len == 2 ? (c & 0x1F) : len == 3 ? (c & 0x0F) : (c & 0x07);
            for (int k = 1; k < len && i + k < text.size(); k++) { cp = (cp << 6) | ((unsigned char)text[i + k] & 0x3F); }
            i += len;
            int adv, lsb;
            stbtt_GetCodepointHMetrics(font, (int)cp, &adv, &lsb);
            if (!measureOnly) {
                int bw, bh, xo, yo;
                unsigned char* bm = stbtt_GetCodepointBitmap(font, scale, scale, (int)cp, &bw, &bh, &xo, &yo);
                if (bm) {
                    int gx = (int)std::lround(pen) + xo, gy = y + yo;
                    for (int yy = 0; yy < bh; yy++) {
                        for (int xx = 0; xx < bw; xx++) { cv.blend(gx + xx, gy + yy, cr, cg, cb, bm[yy * bw + xx] / 255.0); }
                    }
                    stbtt_FreeBitmap(bm, nullptr);
                }
            }
            pen += adv * scale;
        }
        return (int)std::lround(pen) - x;
    }

    std::string legendPng(const Scale& s) const {
        std::vector<unsigned char> ttf;
        stbtt_fontinfo fi;
        Canvas cv = legendCanvas(s, loadFont(fontPath, ttf, fi));
        return encodePng(cv.px, cv.w, cv.h);
    }

    // Dark box: title, color bar red -> green with the end values, black dot "no service"
    static double drivemapNice(double v) {
        if (v <= 0) { return 1; }
        double e = std::pow(10.0, std::floor(std::log10(v)));
        double f = v / e;
        return (f >= 5 ? 5 : f >= 2 ? 2 : 1) * e;
    }

    static Canvas legendCanvas(const Scale& s, const stbtt_fontinfo* font) { return legendCanvasScaled(s, font, 1.0f); }

    static Canvas legendCanvasScaled(const Scale& s, const stbtt_fontinfo* font, float k) {
        auto K = [k](double v) { return (int)std::lround(v * k); };
        const int W = K(240), H = K(font ? 92 : 40), PAD = K(10);
        Canvas cv(W, H);
        cv.rect(0, 0, W, H, 24, 26, 28, 0.85);
        int y = PAD;
        if (font) {
            drawText(cv, font, 18 * k, PAD, y + K(15), s.title);
            y += K(24);
        }
        int bx0 = PAD, bx1 = W - PAD;
        for (int x = bx0; x < bx1; x++) {
            int r, g, b;
            double t = (double)(x - bx0) / (bx1 - bx0 - 1);
            if (s.steps) { t = t < 1.0 / 3 ? 0.0 : t < 2.0 / 3 ? 0.5 : 1.0; }
            ramp(t, r, g, b);
            cv.rect(x, y, x + 1, y + K(14), r, g, b, 1.0);
        }
        y += K(14);
        if (font) {
            std::string l = s.loText + (s.unit.empty() || s.title.find('(') != std::string::npos ? "" : " " + s.unit);
            std::string h = s.hiText;
            drawText(cv, font, 15 * k, bx0, y + K(16), l);
            Canvas dummy(1, 1);
            int hw = drawText(dummy, font, 15 * k, 0, 0, h, true);
            drawText(cv, font, 15 * k, bx1 - hw, y + K(16), h);
            y += K(22);
            // black dot with a light rim: no service
            cv.disc(PAD + 6.5 * k, y + 9.5 * k, 6.5 * k, 0, 0, 0, 200, 200, 200, 1.5 * k);
            drawText(cv, font, 15 * k, PAD + K(18), y + K(14), "no service");
        }
        return cv;
    }

    // ---- minimal ZIP writer (deflate via stb_image_write, PNGs stored) ----
    struct ZipEntry {
        std::string name;
        std::string data;
        bool deflate;
    };

    static uint32_t crc32(const std::string& d) {
        static uint32_t table[256];
        static bool init = false;
        if (!init) {
            for (uint32_t i = 0; i < 256; i++) {
                uint32_t c = i;
                for (int k = 0; k < 8; k++) { c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1; }
                table[i] = c;
            }
            init = true;
        }
        uint32_t c = 0xFFFFFFFFu;
        for (unsigned char ch : d) { c = table[(c ^ ch) & 0xFF] ^ (c >> 8); }
        return c ^ 0xFFFFFFFFu;
    }

    static void put16(std::string& o, uint16_t v) { o += (char)(v & 0xFF); o += (char)(v >> 8); }
    static void put32(std::string& o, uint32_t v) { put16(o, (uint16_t)(v & 0xFFFF)); put16(o, (uint16_t)(v >> 16)); }

    static bool writeZip(const std::string& path, const std::vector<ZipEntry>& entries) {
        time_t now = time(nullptr);
        struct tm lt;
        isoLocalTm(now, lt);
        uint16_t dosTime = (uint16_t)((lt.tm_hour << 11) | (lt.tm_min << 5) | (lt.tm_sec / 2));
        uint16_t dosDate = (uint16_t)(((lt.tm_year - 80) << 9) | ((lt.tm_mon + 1) << 5) | lt.tm_mday);
        std::string out, central;
        for (auto& e : entries) {
            std::string payload = e.data;
            uint16_t method = 0;
            if (e.deflate) {
                int zlen = 0;
                unsigned char* z = stbi_zlib_compress((unsigned char*)e.data.data(), (int)e.data.size(), &zlen, 8);
                // zlib stream = 2 header bytes + raw deflate + 4 bytes Adler-32; ZIP wants raw deflate
                if (z && zlen > 6) {
                    payload.assign((const char*)z + 2, zlen - 6);
                    method = 8;
                }
                if (z) { STBIW_FREE(z); }
            }
            uint32_t crc = crc32(e.data);
            uint32_t offset = (uint32_t)out.size();
            put32(out, 0x04034b50);
            put16(out, 20);
            put16(out, 0);
            put16(out, method);
            put16(out, dosTime);
            put16(out, dosDate);
            put32(out, crc);
            put32(out, (uint32_t)payload.size());
            put32(out, (uint32_t)e.data.size());
            put16(out, (uint16_t)e.name.size());
            put16(out, 0);
            out += e.name;
            out += payload;
            put32(central, 0x02014b50);
            put16(central, 20);
            put16(central, 20);
            put16(central, 0);
            put16(central, method);
            put16(central, dosTime);
            put16(central, dosDate);
            put32(central, crc);
            put32(central, (uint32_t)payload.size());
            put32(central, (uint32_t)e.data.size());
            put16(central, (uint16_t)e.name.size());
            put16(central, 0);
            put16(central, 0);
            put16(central, 0);
            put16(central, 0);
            put32(central, 0);
            put32(central, offset);
            central += e.name;
        }
        uint32_t cdOffset = (uint32_t)out.size();
        out += central;
        put32(out, 0x06054b50);
        put16(out, 0);
        put16(out, 0);
        put16(out, (uint16_t)entries.size());
        put16(out, (uint16_t)entries.size());
        put32(out, (uint32_t)central.size());
        put32(out, cdOffset);
        put16(out, 0);
        return writeText(path, out);
    }

    static bool writeText(const std::string& path, const std::string& data) {
        FILE* f = fopen(path.c_str(), "wb");
        if (!f) { return false; }
        bool ok = fwrite(data.data(), 1, data.size(), f) == data.size();
        ok = (fclose(f) == 0) && ok;
        return ok;
    }

    static std::string fmtNum(double v) {
        char b[32];
        snprintf(b, sizeof(b), "%g", v);
        return b;
    }

    static std::vector<std::string> split(const std::string& l) {
        std::vector<std::string> out;
        std::string cur;
        for (char c : l) {
            if (c == ';') { out.push_back(cur); cur.clear(); }
            else if (c != '\r') { cur += c; }
        }
        out.push_back(cur);
        return out;
    }

    static void ramp(double t, int& r, int& g, int& b) { cov::ramp(t, r, g, b); }

    static std::string xmlEscape(const std::string& s) {
        std::string o;
        for (char c : s) {
            switch (c) {
            case '&': o += "&amp;"; break;
            case '<': o += "&lt;"; break;
            case '>': o += "&gt;"; break;
            case '"': o += "&quot;"; break;
            default: o += c;
            }
        }
        return o;
    }

    template <class P, class N>
    bool writeHtml(const std::string& path, const std::vector<P>& pts, N norm, const std::string& name,
                   const std::string& unit, double lo, double hi, bool logScale) {
        FILE* f = fopen(path.c_str(), "w");
        if (!f) { return false; }
        fprintf(f, "<!doctype html>\n<html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
                   "<title>TETRA coverage - %s</title>\n"
                   "<link rel=\"stylesheet\" href=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.css\">\n"
                   "<script src=\"https://unpkg.com/leaflet@1.9.4/dist/leaflet.js\"></script>\n"
                   "<script src=\"https://unpkg.com/leaflet.heat@0.2.0/dist/leaflet-heat.js\"></script>\n"
                   "<style>html,body,#map{height:100%%;margin:0}.lg{background:#fff;padding:6px 8px;font:12px sans-serif;border-radius:4px;box-shadow:0 1px 4px #0004}"
                   ".bar{width:160px;height:10px;background:linear-gradient(90deg,#e62828,#e6e628,#28e628);margin:4px 0}</style>\n"
                   "</head><body><div id=\"map\"></div>\n<script>\n",
                name.c_str());
        // data: [lat, lon, t(0..1) or -1 for no service, label]
        fprintf(f, "var pts=[\n");
        for (auto& p : pts) {
            std::string lbl = p.label;
            for (auto& c : lbl) { if (c == '"' || c == '\\') { c = '\''; } }
            fprintf(f, "[%.6f,%.6f,%.3f,\"%s\"],\n", p.lat, p.lon, p.service ? norm(p.v) : -1.0, lbl.c_str());
        }
        fprintf(f, "];\n");
        fprintf(f, "var map=L.map('map');\n"
                   "L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{maxZoom:19,attribution:'&copy; OpenStreetMap contributors'}).addTo(map);\n"
                   "function col(t){if(t<0)return '#000';var r,g;if(t<0.5){r=230;g=40+400*t}else{r=230-400*(t-0.5);g=230}return 'rgb('+Math.round(r)+','+Math.round(g)+',40)'}\n"
                   "var dots=L.layerGroup(),b=[];\n"
                   "pts.forEach(function(p){b.push([p[0],p[1]]);L.circleMarker([p[0],p[1]],{radius:5,weight:0,fillOpacity:0.85,fillColor:col(p[2])}).bindPopup(p[3]).addTo(dots)});\n"
                   "dots.addTo(map);map.fitBounds(b,{padding:[20,20]});\n"
                   "var heat=L.heatLayer(pts.filter(function(p){return p[2]>=0}).map(function(p){return [p[0],p[1],0.2+0.8*p[2]]}),{radius:18,blur:20,maxZoom:17,max:1.0,"
                   "gradient:{0.2:'#e62828',0.6:'#e6e628',1.0:'#28e628'}});\n"
                   "var nosvc=L.layerGroup();pts.forEach(function(p){if(p[2]<0)L.circleMarker([p[0],p[1]],{radius:4,weight:0,fillOpacity:0.9,fillColor:'#000'}).addTo(nosvc)});\n"
                   "L.control.layers(null,{'Points':dots,'Heatmap':heat,'No service only':nosvc}).addTo(map);\n"
                   "var lg=L.control({position:'bottomright'});lg.onAdd=function(){var d=L.DomUtil.create('div','lg');"
                   "d.innerHTML='<b>%s</b><div class=bar></div>%s%g %s &nbsp;&ndash;&nbsp; %g %s<br><span style=\"color:#000\">&#9679;</span> no service';return d};lg.addTo(map);\n",
                name.c_str(), logScale ? "" : "", lo, unit.c_str(), hi, unit.c_str());
        fprintf(f, "</script></body></html>\n");
        fclose(f);
        return true;
    }

    void setStatus(const std::string& s) {
        std::lock_guard<std::mutex> lck(mtx);
        statusText = s;
    }

    std::mutex mtx;
    std::string statusText;
    std::atomic<bool> isBusy { false };
    std::thread worker;
};
