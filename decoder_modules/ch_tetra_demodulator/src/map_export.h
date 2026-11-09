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
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include "coverage_scale.h"
#include "iso_time.h"

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
            r.crcPct = ok + fail > 0 ? 100.0 * fail / (ok + fail) : 100.0;
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
            char lb[256];
            snprintf(lb, sizeof(lb), "%s | Level %.1f %s | MER %.1f dB | BER %s | CRC errors %.1f %% | MCC %s MNC %s CC %s%s",
                     r.time.c_str(), r.level, useDbm ? "dBm" : "dBFS", r.mer, r.berS.c_str(), r.crcPct,
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
        default: return cov::quality(r.crcPct, crcLimit);
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
            snprintf(b, sizeof(b), "%.1f %%", r.crcPct);
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
    };

    // Draws UTF-8 text with its baseline at y; returns the advance in pixels (0 without font)
    static int drawText(Canvas& cv, const stbtt_fontinfo* font, float px, int x, int y, const std::string& text, bool measureOnly = false) {
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
                        for (int xx = 0; xx < bw; xx++) { cv.blend(gx + xx, gy + yy, 240, 240, 240, bm[yy * bw + xx] / 255.0); }
                    }
                    stbtt_FreeBitmap(bm, nullptr);
                }
            }
            pen += adv * scale;
        }
        return (int)std::lround(pen) - x;
    }

    // Dark box: title, color bar red -> green with the end values, black dot "no service"
    std::string legendPng(const Scale& s) const {
        std::vector<unsigned char> ttf;
        stbtt_fontinfo fi;
        const stbtt_fontinfo* font = nullptr;
        if (!fontPath.empty()) {
            std::ifstream f(fontPath, std::ios::binary);
            if (f) {
                ttf.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
                if (!ttf.empty() && stbtt_InitFont(&fi, ttf.data(), stbtt_GetFontOffsetForIndex(ttf.data(), 0))) { font = &fi; }
            }
        }
        const int W = 240, H = font ? 92 : 40, PAD = 10;
        Canvas cv(W, H);
        cv.rect(0, 0, W, H, 24, 26, 28, 0.85);
        int y = PAD;
        if (font) {
            drawText(cv, font, 18, PAD, y + 15, s.title);
            y += 24;
        }
        int bx0 = PAD, bx1 = W - PAD;
        for (int x = bx0; x < bx1; x++) {
            int r, g, b;
            double t = (double)(x - bx0) / (bx1 - bx0 - 1);
            if (s.steps) { t = t < 1.0 / 3 ? 0.0 : t < 2.0 / 3 ? 0.5 : 1.0; }
            ramp(t, r, g, b);
            cv.rect(x, y, x + 1, y + 14, r, g, b, 1.0);
        }
        y += 14;
        if (font) {
            std::string l = s.loText + (s.unit.empty() || s.title.find('(') != std::string::npos ? "" : " " + s.unit);
            std::string h = s.hiText;
            drawText(cv, font, 15, bx0, y + 16, l);
            Canvas dummy(1, 1);
            int hw = drawText(dummy, font, 15, 0, 0, h, true);
            drawText(cv, font, 15, bx1 - hw, y + 16, h);
            y += 22;
            // black dot with a light rim: no service
            int cx = PAD + 6, cy = y + 9;
            for (int yy = -7; yy <= 7; yy++) {
                for (int xx = -7; xx <= 7; xx++) {
                    double d = std::sqrt((double)(xx * xx + yy * yy));
                    if (d <= 6.5) { cv.blend(cx + xx, cy + yy, d > 5.0 ? 200 : 0, d > 5.0 ? 200 : 0, d > 5.0 ? 200 : 0, 1.0); }
                }
            }
            drawText(cv, font, 15, PAD + 18, cy + 5, "no service");
        }
        return encodePng(cv.px, W, H);
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
