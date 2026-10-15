#pragma once
// Measurement report: one self-contained HTML file (charts as inline SVG,
// logo embedded) with the results of the session. Print it from the browser
// to get a PDF.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iterator>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "trend.h"
#include "burst_dsp.h"
#include "tetra_sigproc.h"
#include "freq_quality.h"

#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#pragma comment(lib, "shell32.lib")
#endif

struct TetraReportData {
    std::string edition, version, title = "TETRA measurement report";
    std::string logoPath;                 // PNG, embedded as data URI
    std::string notes;                    // user text
    // cell
    bool cellValid = false;
    int mcc = 0, mnc = 0, cc = 0;
    double dlFreq = 0, ulFreq = 0, vfoFreq = 0;
    bool levelCalibrated = false;
    // live values
    std::vector<TetraTrendPoint> trend;   // the span of the report
    uint32_t dlUseTotal[4][4] = {};
    uint32_t ulUseTotal[4][3] = {};
    uint32_t syncLostTotal = 0, crcOkTotal = 0, crcFailTotal = 0;
    double berTotal = -1.0;
    // TX analysis
    bool txaValid = false;
    double txaChan = 0, txaAcp[6] = {}, txaAcpFloor[6] = {}, txaObw = 0, txaObwIdeal = 0, txaBw26 = 0, txaBw26Ideal = 0, txaLeak = 0;
    bool txaLeakBelow = true;
    // bursts / channel
    bool burstValid = false;
    double burstFreq = 0;
    TetraBurstResult burst;
    bool mpValid = false;
    tetra_sp::MultipathResult mp;
    int mpSlots = 0;
    // carriers: label, MHz, level, MER (NaN), sync
    struct Carrier { std::string label; double mhz; float level, mer; bool sync; };
    std::vector<Carrier> carriers;
    float mainLevel = NAN;
    // stations / events
    int stations = 0, groups = 0, traffic = 0;
    std::vector<std::pair<time_t, std::string>> events;
    float crcLimit = 1.0f;
};

class TetraReport {
public:
    // writes the report; returns the path or an error text (starting with "!")
    static std::string write(const std::string& dir, const TetraReportData& d) {
        time_t now = time(nullptr);
        struct tm lt;
        localTm(now, lt);
        char name[64];
        strftime(name, sizeof(name), "tetra_report_%Y%m%d_%H%M%S.html", &lt);
        std::string path = dir + "/" + name;
        std::ofstream f(path);
        if (!f) { return "!Cannot write " + path; }
        f << build(d, now);
        return path;
    }

    static void openInBrowser(const std::string& path) {
#ifdef _WIN32
        ShellExecuteA(NULL, "open", path.c_str(), NULL, NULL, SW_SHOWNORMAL);
#elif defined(__APPLE__)
        std::string cmd = "open '" + path + "' &";
        if (system(cmd.c_str())) {}
#else
        // without the AppImage's library path, so the browser uses its own libraries
        std::string cmd = "env -u LD_LIBRARY_PATH xdg-open '" + path + "' >/dev/null 2>&1 &";
        if (system(cmd.c_str())) {}
#endif
    }

    static std::string build(const TetraReportData& d, time_t now) {
        std::ostringstream o;
        const char* unit = d.levelCalibrated ? "dBm" : "dBFS";
        o << "<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\"><title>" << esc(d.title) << "</title>\n<style>\n"
          << ":root{--ink:#16202b;--mut:#5b6876;--line:#d5dbe2;--acc:#0b4a8b;--bg:#fff;--soft:#f3f6f9}\n"
          << "body{font:14px/1.45 system-ui,-apple-system,'Segoe UI',sans-serif;color:var(--ink);background:var(--bg);margin:0}\n"
          << ".wrap{max-width:960px;margin:0 auto;padding:28px 32px}\n"
          << "header{display:flex;align-items:center;gap:18px;border-bottom:3px solid var(--acc);padding-bottom:14px;margin-bottom:18px}\n"
          << "header img{width:64px;height:64px;border-radius:12px}\nh1{font-size:24px;margin:0;color:var(--acc)}\n"
          << ".sub{color:var(--mut);font-size:13px}\nh2{font-size:17px;margin:26px 0 8px;color:var(--acc);border-bottom:1px solid var(--line);padding-bottom:4px}\n"
          << "table{border-collapse:collapse;width:100%;margin:6px 0 10px;font-variant-numeric:tabular-nums}\n"
          << "th,td{border-bottom:1px solid var(--line);padding:4px 8px;text-align:left}th{background:var(--soft);font-weight:600}\n"
          << "td.n{text-align:right}th+th{text-align:right}table.txt th+th{text-align:left}.kv td:first-child{color:var(--mut);width:40%}\n"
          << ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:10px}\n"
          << ".tile{background:var(--soft);border-radius:8px;padding:10px 12px}.tile b{display:block;font-size:20px;color:var(--ink)}.tile span{color:var(--mut);font-size:12px}\n"
          << "svg{width:100%;height:auto;background:var(--soft);border-radius:6px}\n.note{color:var(--mut);font-size:12px}\n"
          << "@media print{.wrap{padding:0}h2{break-after:avoid}svg,table{break-inside:avoid}}\n</style></head><body><div class=\"wrap\">\n";
        o << "<header>";
        std::string logo = dataUri(d.logoPath);
        if (!logo.empty()) { o << "<img src=\"" << logo << "\" alt=\"\">"; }
        o << "<div><h1>" << esc(d.title) << "</h1><div class=\"sub\">" << esc(d.edition) << " v" << esc(d.version) << " &middot; created "
          << fmtTime(now, true) << "</div></div></header>\n";
        if (!d.notes.empty()) { o << "<p>" << esc(d.notes) << "</p>\n"; }

        // ---- summary ----
        Stats lv = stats(d.trend, [](const TetraTrendPoint& p) { return p.level; });
        Stats mer = stats(d.trend, [](const TetraTrendPoint& p) { return p.mer; });
        Stats evm = stats(d.trend, [](const TetraTrendPoint& p) { return p.evm; });
        Stats crc = stats(d.trend, [](const TetraTrendPoint& p) { return p.crcFail; });
        Stats ppb = stats(d.trend, [](const TetraTrendPoint& p) { return p.ppb; });
        o << "<h2>Summary</h2>\n";
        if (!d.trend.empty()) {
            o << "<p class=\"note\">Period: " << fmtTime(d.trend.front().t, true) << " &ndash; " << fmtTime(d.trend.back().t, true)
              << " (" << d.trend.size() << " one-second values)</p>\n";
        }
        o << "<div class=\"grid\">";
        tile(o, "Level (mean)", lv.n ? num(lv.mean, 1) + " " + unit : "&ndash;", lv.n ? "min " + num(lv.min, 1) + ", max " + num(lv.max, 1) : "");
        tile(o, "MER (mean)", mer.n ? num(mer.mean, 1) + " dB" : "&ndash;", mer.n ? "min " + num(mer.min, 1) + " dB" : "");
        tile(o, "EVM rms (mean)", evm.n ? num(evm.mean, 1) + " %" : "&ndash;", "ETSI limit 10 %");
        tile(o, "CRC errors", crc.n ? num(crc.mean, 2) + " %" : "&ndash;", crc.n ? "worst second " + num(crc.max, 1) + " %" : "");
        tile(o, "Sync losses", std::to_string(d.syncLostTotal), d.berTotal >= 0 ? "raw BER " + sci(d.berTotal) : "");
        tile(o, "Carrier vs. SYSINFO", ppb.n ? num(ppb.mean, 1) + " ppb" : "&ndash;", "BS + SDR error, ETSI BS +-200 ppb");
        o << "</div>\n";

        // ---- cell ----
        o << "<h2>Cell</h2><table class=\"kv\">";
        if (d.cellValid) {
            char b[64];
            snprintf(b, sizeof(b), "%03d / %03d", d.mcc, d.mnc);
            row(o, "MCC / MNC", b);
            snprintf(b, sizeof(b), "0x%02X (%d)", d.cc, d.cc);
            row(o, "Colour code", b);
        }
        if (d.dlFreq > 1e6) { row(o, "Downlink (SYSINFO)", num(d.dlFreq / 1e6, 5) + " MHz"); }
        if (d.ulFreq > 1e6) { row(o, "Uplink (SYSINFO)", num(d.ulFreq / 1e6, 5) + " MHz"); }
        if (d.vfoFreq > 1e6) { row(o, "TETRA VFO", num(d.vfoFreq / 1e6, 5) + " MHz"); }
        row(o, "Stations / groups seen", std::to_string(d.stations) + " / " + std::to_string(d.groups));
        row(o, "Traffic entries", std::to_string(d.traffic));
        o << "</table>\n";

        // ---- trends ----
        if (d.trend.size() > 2) {
            o << "<h2>Trends</h2>\n";
            o << svgSeries(d.trend, [](const TetraTrendPoint& p) { return p.level; }, std::string("Level ") + unit, "#0b4a8b");
            o << svgSeries(d.trend, [](const TetraTrendPoint& p) { return p.mer; }, "MER dB", "#1b8a3a");
            o << svgSeries(d.trend, [](const TetraTrendPoint& p) { return p.crcFail; }, "CRC errors %", "#c0392b");
            bool anyTs = false;
            for (auto& p : d.trend) { if (!std::isnan(p.tsTraffic[1])) { anyTs = true; break; } }
            if (anyTs) { o << svgTimeslots(d.trend); }
        }

        // ---- timeslots ----
        uint32_t anyUse = 0;
        for (int t = 0; t < 4; t++) { for (int c = 0; c < 4; c++) { anyUse += d.dlUseTotal[t][c]; } }
        if (anyUse) {
            o << "<h2>Timeslot occupancy</h2><table><tr><th>TS</th><th>Traffic</th><th>Assigned control</th><th>Common control</th><th>Free</th><th>UL traffic</th></tr>\n";
            for (int t = 0; t < 4; t++) {
                uint32_t n = 0, nu = 0;
                for (int c = 0; c < 4; c++) { n += d.dlUseTotal[t][c]; }
                for (int c = 0; c < 3; c++) { nu += d.ulUseTotal[t][c]; }
                o << "<tr><td>TS" << t + 1 << "</td>";
                int order[4] = { 3, 1, 2, 0 };
                for (int c : order) { o << "<td class=\"n\">" << (n ? num(100.0 * d.dlUseTotal[t][c] / n, 1) + " %" : "&ndash;") << "</td>"; }
                o << "<td class=\"n\">" << (nu ? num(100.0 * d.ulUseTotal[t][2] / nu, 1) + " %" : "&ndash;") << "</td></tr>\n";
            }
            o << "</table><p class=\"note\">From the ACCESS-ASSIGN PDU of the downlink bursts, frames 1&ndash;17.</p>\n";
        }

        // ---- reception quality vs level ----
        auto q = tetra_eval::qualityVsLevel(d.trend, d.crcLimit);
        if (q.bins.size() >= 3) {
            o << "<h2>Reception quality vs. level</h2>\n";
            if (!std::isnan(q.thresholdLevel)) {
                o << "<p>Reception limit: <b>" << num(q.thresholdLevel, 0) << " " << unit << "</b> (CRC errors at most " << num(d.crcLimit, 1)
                  << " % above this level).</p>\n";
            }
            o << svgQuality(q, d.crcLimit, unit);
        }

        // ---- frequency stability ----
        {
            std::vector<double> y;
            time_t prev = 0;
            for (auto& p : d.trend) {
                if (prev && p.t > prev + 1) { for (time_t t = prev + 1; t < p.t && y.size() < 200000; t++) { y.push_back(NAN); } }
                y.push_back(p.ppb);
                prev = p.t;
            }
            size_t run = 0;
            auto ad = tetra_eval::allanDeviation(y, run);
            if (!ad.empty()) {
                o << "<h2>Frequency stability</h2><p class=\"note\">Overlapping Allan deviation of the received carrier vs. the SYSINFO frequency over "
                  << run << " s without gaps. Includes the SDR's reference &ndash; meaningful with a GPS-disciplined SDR.</p>\n"
                  << "<table><tr><th>&tau; (s)</th>";
                for (auto& a : ad) { o << "<th>" << num(a.tau, 0) << "</th>"; }
                o << "</tr><tr><td>ADEV (ppb)</td>";
                for (auto& a : ad) { o << "<td class=\"n\">" << num(a.adev, a.adev < 10 ? 2 : 1) << "</td>"; }
                o << "</tr></table>\n";
            }
        }

        // ---- TX analysis ----
        if (d.txaValid) {
            o << "<h2>TX analysis (spectrum)</h2><table class=\"kv\">";
            row(o, "Channel power", num(d.txaChan, 1) + " dBFS");
            row(o, "Occupied bandwidth (99 %)", num(d.txaObw / 1e3, 2) + " kHz (ideal " + num(d.txaObwIdeal / 1e3, 2) + " kHz)");
            row(o, "-26 dB bandwidth", num(d.txaBw26 / 1e3, 2) + " kHz (ideal " + num(d.txaBw26Ideal / 1e3, 2) + " kHz)");
            row(o, "Carrier leakage", (d.txaLeakBelow ? "&lt; " : "") + num(d.txaLeak, 1) + " dBc");
            o << "</table><table><tr><th>Adjacent channel</th><th>-75 kHz</th><th>-50 kHz</th><th>-25 kHz</th><th>+25 kHz</th><th>+50 kHz</th><th>+75 kHz</th></tr><tr><td>ACP dBc</td>";
            for (int i = 0; i < 6; i++) { o << "<td class=\"n\">" << num(d.txaAcp[i], 1) << "</td>"; }
            o << "</tr><tr><td>floor dBc</td>";
            for (int i = 0; i < 6; i++) { o << "<td class=\"n\">" << num(d.txaAcpFloor[i], 1) << "</td>"; }
            o << "</tr></table>\n";
        }

        // ---- bursts ----
        if (d.burstValid && d.burst.state >= 1) {
            const TetraBurstResult& b = d.burst;
            o << "<h2>Burst analysis</h2><table class=\"kv\">";
            row(o, "Frequency", num(d.burstFreq / 1e6, 5) + " MHz");
            row(o, "Bursts", std::to_string(b.nFull) + " full slot, " + std::to_string(b.nHalf) + " half slot");
            if (!b.profileDbc.empty()) {
                row(o, "Burst power", num(b.powerDbfs, 1) + " dBFS (average of " + std::to_string(b.averaged) + ")");
                row(o, "Length (-3 dB)", num(b.lengthSym, 1) + " symbols (" + num(b.lengthSym / 18.0, 2) + " ms)");
                row(o, "Ramp up / down (-20 to -1 dBc)", opt(b.riseSym, 1, " sym") + " / " + opt(b.fallSym, 1, " sym"));
                row(o, "Ripple of the useful part", opt(b.ripplePosDb, 2, " dB") + " / " + opt(b.rippleNegDb, 2, " dB"));
                row(o, "Outside the burst", opt(b.offBeforeDbc, 1, " dBc") + " before, " + opt(b.offAfterDbc, 1, " dBc") + " after (floor " + opt(b.floorDbc, 1, " dBc") + ")");
            }
            if (!std::isnan(b.gapDbfs)) { row(o, "Noise between bursts", num(b.gapDbfs, 1) + " dBFS"); }
            if (b.timingValid) { row(o, "Timing vs. downlink", num(b.timingMeanSym, 2) + " &plusmn; " + num(b.timingStdSym, 2) + " symbols after slot start (" + std::to_string(b.timingN) + " bursts)"); }
            o << "</table>\n";
            if (!b.profileDbc.empty()) { o << svgProfile(b); }
            if (b.transValid) {
                o << "<table><tr><th>Ramp spectrum dBc</th><th>-50 kHz</th><th>-25 kHz</th><th>own</th><th>+25 kHz</th><th>+50 kHz</th></tr>\n";
                const char* rn[3] = { "ramp up", "middle", "ramp down" };
                for (int w = 0; w < 3; w++) {
                    o << "<tr><td>" << rn[w] << "</td>";
                    for (int k = 0; k < 5; k++) { o << "<td class=\"n\">" << num(b.transDbc[w][k], 1) << "</td>"; }
                    o << "</tr>\n";
                }
                o << "<tr><td>floor</td>";
                for (int k = 0; k < 5; k++) { o << "<td class=\"n\">" << opt(b.transFloorDbc[k], 1, "") << "</td>"; }
                o << "</tr></table>\n";
            }
        }

        // ---- multipath ----
        if (d.mpValid && d.mp.valid) {
            o << "<h2>Multipath (downlink)</h2><table class=\"kv\">";
            row(o, "Slots evaluated", std::to_string(d.mpSlots));
            row(o, "Strongest echo", std::isnan(d.mp.echoDelayUs) ? std::string("none above -15 dB")
                                                                  : num(d.mp.echoDelayUs, 0) + " &micro;s, " + num(d.mp.echoLevelDb, 1) + " dB");
            row(o, "Extra delay spread", num(d.mp.extraSpreadUs, 1) + " &micro;s");
            o << "</table>\n";
        }

        // ---- carriers ----
        if (!d.carriers.empty()) {
            o << "<h2>Carriers</h2><table><tr><th>Carrier</th><th>MHz</th><th>Level " << unit << "</th><th>vs. main</th><th>MER</th><th>Sync</th></tr>\n";
            for (auto& c : d.carriers) {
                o << "<tr><td>" << esc(c.label) << "</td><td class=\"n\">" << num(c.mhz, 4) << "</td><td class=\"n\">" << num(c.level, 1) << "</td><td class=\"n\">"
                  << (std::isnan(d.mainLevel) ? std::string("&ndash;") : num(c.level - d.mainLevel, 1) + " dB") << "</td><td class=\"n\">"
                  << (std::isnan(c.mer) ? std::string("&ndash;") : num(c.mer, 1) + " dB") << "</td><td>" << (c.sync ? "yes" : "no") << "</td></tr>\n";
            }
            o << "</table>\n";
        }

        // ---- events ----
        if (!d.events.empty()) {
            o << "<h2>Events</h2><table class=\"txt\"><tr><th>Time</th><th>Event</th></tr>\n";
            size_t n0 = d.events.size() > 60 ? d.events.size() - 60 : 0;
            for (size_t i = n0; i < d.events.size(); i++) {
                o << "<tr><td>" << fmtTime(d.events[i].first, false) << "</td><td>" << esc(d.events[i].second) << "</td></tr>\n";
            }
            o << "</table>\n";
        }
        o << "<p class=\"note\" style=\"margin-top:30px\">Measured with " << esc(d.edition) << " v" << esc(d.version)
          << " (amateur radio, experimental). Levels in dBFS are relative to the SDR's full scale unless calibrated.</p>\n";
        o << "</div></body></html>\n";
        return o.str();
    }

private:
    struct Stats { int n = 0; double mean = 0, min = 0, max = 0; };
    template <class F>
    static Stats stats(const std::vector<TetraTrendPoint>& v, F f) {
        Stats s;
        double sum = 0;
        for (auto& p : v) {
            float x = f(p);
            if (std::isnan(x)) { continue; }
            if (!s.n) { s.min = s.max = x; }
            s.min = (std::min)(s.min, (double)x);
            s.max = (std::max)(s.max, (double)x);
            sum += x;
            s.n++;
        }
        if (s.n) { s.mean = sum / s.n; }
        return s;
    }
    static void localTm(time_t t, struct tm& lt) {
#ifdef _WIN32
        localtime_s(&lt, &t);
#else
        localtime_r(&t, &lt);
#endif
    }
    static std::string fmtTime(time_t t, bool date) {
        struct tm lt;
        localTm(t, lt);
        char b[40];
        strftime(b, sizeof(b), date ? "%Y-%m-%d %H:%M:%S" : "%H:%M:%S", &lt);
        return b;
    }
    static std::string num(double v, int dec) {
        if (std::isnan(v)) { return "&ndash;"; }
        char b[48];
        snprintf(b, sizeof(b), "%.*f", dec, v);
        return b;
    }
    static std::string opt(float v, int dec, const char* unit) { return std::isnan(v) ? std::string("&ndash;") : num(v, dec) + unit; }
    static std::string sci(double v) { char b[32]; snprintf(b, sizeof(b), "%.1e", v); return b; }
    static std::string esc(const std::string& s) {
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
    static void row(std::ostringstream& o, const std::string& k, const std::string& v) { o << "<tr><td>" << k << "</td><td>" << v << "</td></tr>"; }
    static void tile(std::ostringstream& o, const std::string& k, const std::string& v, const std::string& sub) {
        o << "<div class=\"tile\"><span>" << k << "</span><b>" << v << "</b><span>" << sub << "</span></div>";
    }
    static std::string dataUri(const std::string& path) {
        if (path.empty()) { return ""; }
        std::ifstream f(path, std::ios::binary);
        if (!f) { return ""; }
        std::string bin((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
        static const char* tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string out = "data:image/png;base64,";
        size_t i = 0;
        for (; i + 2 < bin.size(); i += 3) {
            uint32_t v = ((uint8_t)bin[i] << 16) | ((uint8_t)bin[i + 1] << 8) | (uint8_t)bin[i + 2];
            out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63]; out += tbl[(v >> 6) & 63]; out += tbl[v & 63];
        }
        if (i < bin.size()) {
            uint32_t v = (uint8_t)bin[i] << 16;
            if (i + 1 < bin.size()) { v |= (uint8_t)bin[i + 1] << 8; }
            out += tbl[(v >> 18) & 63]; out += tbl[(v >> 12) & 63];
            out += (i + 1 < bin.size()) ? tbl[(v >> 6) & 63] : '=';
            out += '=';
        }
        return out;
    }

    template <class F>
    static std::string svgSeries(const std::vector<TetraTrendPoint>& v, F f, const std::string& title, const char* col, double lo = NAN, double hi = NAN, int h = 110) {
        const int W = 900, L = 50, R = 10, T = 18, B = 18;
        double t0 = (double)v.front().t, t1 = (std::max)((double)v.back().t, t0 + 1.0);
        double mn = INFINITY, mx = -INFINITY;
        for (auto& p : v) { float x = f(p); if (!std::isnan(x)) { mn = (std::min)(mn, (double)x); mx = (std::max)(mx, (double)x); } }
        if (!(mn <= mx)) { return ""; }
        if (std::isnan(lo)) { lo = mn; hi = mx; if (hi - lo < 1.0) { lo -= 0.5; hi += 0.5; } double pad = 0.08 * (hi - lo); lo -= pad; hi += pad; }
        const int cols = W - L - R;
        std::vector<double> sum(cols, 0.0), cmin(cols, INFINITY), cmax(cols, -INFINITY);
        std::vector<int> n(cols, 0);
        for (auto& p : v) {
            float x = f(p);
            if (std::isnan(x)) { continue; }
            int k = std::clamp((int)(((double)p.t - t0) / (t1 - t0) * cols), 0, cols - 1);
            sum[k] += x; n[k]++;
            cmin[k] = (std::min)(cmin[k], (double)x);
            cmax[k] = (std::max)(cmax[k], (double)x);
        }
        auto Y = [&](double x) { return T + (hi - std::clamp(x, lo, hi)) / (hi - lo) * (h - T - B); };
        std::ostringstream s;
        s << "<svg viewBox=\"0 0 " << W << " " << h << "\" xmlns=\"http://www.w3.org/2000/svg\" font-family=\"sans-serif\" font-size=\"11\">";
        s << "<text x=\"" << L << "\" y=\"12\" fill=\"#16202b\" font-weight=\"600\">" << esc(title) << "</text>";
        for (int i = 0; i <= 2; i++) {
            double val = lo + (hi - lo) * i / 2;
            double y = Y(val);
            s << "<line x1=\"" << L << "\" x2=\"" << W - R << "\" y1=\"" << y << "\" y2=\"" << y << "\" stroke=\"#d5dbe2\"/>";
            s << "<text x=\"" << L - 4 << "\" y=\"" << y + 4 << "\" text-anchor=\"end\" fill=\"#5b6876\">" << num(val, (hi - lo) < 5 ? 1 : 0) << "</text>";
        }
        for (int i = 0; i <= 4; i++) {
            double t = t0 + (t1 - t0) * i / 4;
            s << "<text x=\"" << L + cols * i / 4 << "\" y=\"" << h - 4 << "\" text-anchor=\"" << (i == 0 ? "start" : i == 4 ? "end" : "middle")
              << "\" fill=\"#5b6876\">" << fmtTime((time_t)t, false) << "</text>";
        }
        // min..max band and mean line
        std::ostringstream band, line;
        bool pen = false;
        for (int k = 0; k < cols; k++) {
            if (!n[k]) { pen = false; continue; }
            double x = L + k + 0.5;
            if (cmax[k] > cmin[k]) { band << "M" << x << " " << Y(cmax[k]) << "V" << Y(cmin[k]); }
            line << (pen ? "L" : "M") << x << " " << Y(sum[k] / n[k]);
            pen = true;
        }
        s << "<path d=\"" << band.str() << "\" stroke=\"" << col << "\" stroke-opacity=\"0.25\" fill=\"none\"/>";
        s << "<path d=\"" << line.str() << "\" stroke=\"" << col << "\" stroke-width=\"1.5\" fill=\"none\"/></svg>\n";
        return s.str();
    }

    static std::string svgTimeslots(const std::vector<TetraTrendPoint>& v) {
        const int W = 900, H = 130, L = 50, R = 10, T = 18, B = 18;
        static const char* cols[4] = { "#0f8fa6", "#a3289a", "#d07a10", "#3a8f2a" };
        double t0 = (double)v.front().t, t1 = (std::max)((double)v.back().t, t0 + 1.0);
        const int nc = W - L - R;
        auto Y = [&](double x) { return T + (100.0 - std::clamp(x, 0.0, 100.0)) / 100.0 * (H - T - B); };
        std::ostringstream s;
        s << "<svg viewBox=\"0 0 " << W << " " << H << "\" xmlns=\"http://www.w3.org/2000/svg\" font-family=\"sans-serif\" font-size=\"11\">";
        s << "<text x=\"" << L << "\" y=\"12\" fill=\"#16202b\" font-weight=\"600\">Traffic per timeslot %</text>";
        for (int t = 0; t < 4; t++) { s << "<text x=\"" << L + 170 + 45 * t << "\" y=\"12\" fill=\"" << cols[t] << "\" font-weight=\"600\">TS" << t + 1 << "</text>"; }
        for (int i = 0; i <= 2; i++) {
            double y = Y(50.0 * i);
            s << "<line x1=\"" << L << "\" x2=\"" << W - R << "\" y1=\"" << y << "\" y2=\"" << y << "\" stroke=\"#d5dbe2\"/><text x=\"" << L - 4 << "\" y=\"" << y + 4
              << "\" text-anchor=\"end\" fill=\"#5b6876\">" << 50 * i << "</text>";
        }
        for (int i = 0; i <= 4; i++) {
            s << "<text x=\"" << L + nc * i / 4 << "\" y=\"" << H - 4 << "\" text-anchor=\"" << (i == 0 ? "start" : i == 4 ? "end" : "middle") << "\" fill=\"#5b6876\">"
              << fmtTime((time_t)(t0 + (t1 - t0) * i / 4), false) << "</text>";
        }
        for (int t = 0; t < 4; t++) {
            std::vector<double> sum(nc, 0.0);
            std::vector<int> n(nc, 0);
            for (auto& p : v) {
                if (std::isnan(p.tsTraffic[t])) { continue; }
                int k = std::clamp((int)(((double)p.t - t0) / (t1 - t0) * nc), 0, nc - 1);
                sum[k] += p.tsTraffic[t]; n[k]++;
            }
            // smooth over 5 pixels so the four lines stay readable
            std::ostringstream line;
            bool pen = false;
            for (int k = 0; k < nc; k++) {
                double a = 0; int c = 0;
                for (int j = (std::max)(0, k - 2); j <= (std::min)(nc - 1, k + 2); j++) { a += sum[j]; c += n[j]; }
                if (!c) { pen = false; continue; }
                line << (pen ? "L" : "M") << L + k + 0.5 << " " << Y(a / c);
                pen = true;
            }
            s << "<path d=\"" << line.str() << "\" stroke=\"" << cols[t] << "\" stroke-width=\"1.4\" fill=\"none\"/>";
        }
        s << "</svg>\n";
        return s.str();
    }

    static std::string svgQuality(const tetra_eval::QualityCurve& q, float limit, const char* unit) {
        const int W = 900, H = 220, L = 50, R = 50, T = 18, B = 22;
        float x0 = q.bins.front().level - 0.5f, x1 = q.bins.back().level + 0.5f;
        if (x1 - x0 < 10) { x1 = x0 + 10; }
        auto X = [&](double l) { return L + (l - x0) / (x1 - x0) * (W - L - R); };
        auto Yc = [&](double pct) { return T + (2.0 - std::log10(std::clamp(pct, 0.01, 100.0))) / 4.0 * (H - T - B); };
        auto Ym = [&](double mer) { return T + (40.0 - std::clamp(mer, 0.0, 40.0)) / 40.0 * (H - T - B); };
        std::ostringstream s;
        s << "<svg viewBox=\"0 0 " << W << " " << H << "\" xmlns=\"http://www.w3.org/2000/svg\" font-family=\"sans-serif\" font-size=\"11\">";
        s << "<text x=\"" << L << "\" y=\"12\" fill=\"#c0392b\" font-weight=\"600\">CRC errors % (left, log)</text>"
          << "<text x=\"" << W - R << "\" y=\"12\" text-anchor=\"end\" fill=\"#1b8a3a\" font-weight=\"600\">MER dB (right)</text>";
        const double pcts[5] = { 0.01, 0.1, 1, 10, 100 };
        for (double pc : pcts) {
            s << "<line x1=\"" << L << "\" x2=\"" << W - R << "\" y1=\"" << Yc(pc) << "\" y2=\"" << Yc(pc) << "\" stroke=\"#d5dbe2\"/><text x=\"" << L - 4 << "\" y=\"" << Yc(pc) + 4
              << "\" text-anchor=\"end\" fill=\"#5b6876\">" << pc << "</text>";
        }
        for (int m = 0; m <= 40; m += 10) {
            s << "<text x=\"" << W - R + 4 << "\" y=\"" << Ym(m) + 4 << "\" fill=\"#5b6876\">" << m << "</text>";
        }
        int step = (x1 - x0) > 40 ? 10 : 5;
        for (int l = (int)std::ceil(x0 / step) * step; l <= x1; l += step) {
            s << "<text x=\"" << X(l) << "\" y=\"" << H - 6 << "\" text-anchor=\"middle\" fill=\"#5b6876\">" << l << "</text>";
        }
        s << "<text x=\"" << W - R << "\" y=\"" << H - 6 << "\" text-anchor=\"end\" fill=\"#5b6876\">" << unit << "</text>";
        s << "<line x1=\"" << L << "\" x2=\"" << W - R << "\" y1=\"" << Yc(limit) << "\" y2=\"" << Yc(limit) << "\" stroke=\"#c0392b\" stroke-dasharray=\"4 3\" stroke-opacity=\"0.6\"/>";
        if (!std::isnan(q.thresholdLevel)) {
            s << "<line x1=\"" << X(q.thresholdLevel) << "\" x2=\"" << X(q.thresholdLevel) << "\" y1=\"" << T << "\" y2=\"" << H - B << "\" stroke=\"#0b4a8b\" stroke-width=\"1.5\"/>";
        }
        std::ostringstream lc, lm;
        bool pc = false, pm = false;
        for (auto& b : q.bins) {
            lc << (pc ? "L" : "M") << X(b.level) << " " << Yc(b.crcFail); pc = true;
            s << "<circle cx=\"" << X(b.level) << "\" cy=\"" << Yc(b.crcFail) << "\" r=\"2.5\" fill=\"#c0392b\"/>";
            if (!std::isnan(b.mer)) { lm << (pm ? "L" : "M") << X(b.level) << " " << Ym(b.mer); pm = true; }
        }
        s << "<path d=\"" << lc.str() << "\" stroke=\"#c0392b\" fill=\"none\"/><path d=\"" << lm.str() << "\" stroke=\"#1b8a3a\" stroke-width=\"1.5\" fill=\"none\"/></svg>\n";
        return s.str();
    }

    static std::string svgProfile(const TetraBurstResult& b) {
        const int W = 900, H = 220, L = 50, R = 10, T = 14, B = 22;
        const int n = (int)b.profileDbc.size();
        double xs0 = -b.preS * 18000.0, xs1 = (n / b.fs - b.preS) * 18000.0;
        auto X = [&](double s) { return L + (s - xs0) / (xs1 - xs0) * (W - L - R); };
        auto Y = [&](double db) { return T + (5.0 - std::clamp(db, -70.0, 5.0)) / 75.0 * (H - T - B); };
        std::ostringstream s;
        s << "<svg viewBox=\"0 0 " << W << " " << H << "\" xmlns=\"http://www.w3.org/2000/svg\" font-family=\"sans-serif\" font-size=\"11\">";
        for (int db = 0; db >= -70; db -= 10) {
            s << "<line x1=\"" << L << "\" x2=\"" << W - R << "\" y1=\"" << Y(db) << "\" y2=\"" << Y(db) << "\" stroke=\"#d5dbe2\"/>";
            s << "<text x=\"" << L - 4 << "\" y=\"" << Y(db) + 4 << "\" text-anchor=\"end\" fill=\"#5b6876\">" << db << "</text>";
        }
        for (int sy = 0; sy <= (int)xs1; sy += 50) {
            s << "<text x=\"" << X(sy) << "\" y=\"" << H - 6 << "\" text-anchor=\"middle\" fill=\"#5b6876\">" << sy << "</text>";
        }
        s << "<text x=\"" << W - R << "\" y=\"" << T + 12 << "\" text-anchor=\"end\" fill=\"#5b6876\">burst profile: dBc over symbols</text>";
        std::ostringstream line;
        int cols = W - L - R;
        for (int c = 0; c < cols; c++) {
            int ka = c * n / cols, kb = (std::max)(ka + 1, (c + 1) * n / cols);
            float mx = -200.0f;
            for (int k = ka; k < kb && k < n; k++) { mx = (std::max)(mx, b.profileDbc[k]); }
            line << (c ? "L" : "M") << L + c + 0.5 << " " << Y(mx);
        }
        s << "<path d=\"" << line.str() << "\" stroke=\"#0b4a8b\" stroke-width=\"1.5\" fill=\"none\"/></svg>\n";
        return s.str();
    }
};
