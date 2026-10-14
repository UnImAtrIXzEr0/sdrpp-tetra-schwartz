#pragma once
// Coverage map export from a measurement CSV:
//  - KML for Google Earth (colored points, one per second with position)
//  - self-contained HTML with OpenStreetMap (Leaflet), colored points,
//    optional heatmap layer and legend
// Runs in a background thread; never touches the DSP chain.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

class CoverageMapExporter {
public:
    enum Metric { METRIC_LEVEL = 0, METRIC_MER = 1, METRIC_BER = 2 };

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

    // Synchronous core (public for tests)
    std::string run(const std::string& csvPath, Metric metric, bool onlyLockedCell) {
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
        if (cLat < 0 || cLon < 0) { return "CSV has no position columns (log from v0.5.0 or newer needed)"; }

        struct Pt {
            double lat, lon;
            double v;      // metric value
            bool service;  // cell received (and matching the lock, if requested)
            std::string label;
        };
        std::vector<Pt> pts;
        bool useDbm = false;
        while (std::getline(in, line)) {
            auto f = split(line);
            auto get = [&](int c) { return (c >= 0 && c < (int)f.size()) ? f[c] : std::string(); };
            if (cFix >= 0 && get(cFix) != "1") { continue; }
            std::string sLat = get(cLat), sLon = get(cLon);
            if (sLat.empty() || sLon.empty()) { continue; }
            Pt p;
            p.lat = atof(sLat.c_str());
            p.lon = atof(sLon.c_str());
            if (p.lat == 0.0 && p.lon == 0.0) { continue; }
            int state = atoi(get(cState).c_str());
            std::string match = get(cMatch);
            p.service = (state == 2) && !(onlyLockedCell && match == "0");
            std::string dbm = get(cDbm);
            double level;
            if (!dbm.empty()) { level = atof(dbm.c_str()); useDbm = true; }
            else { level = atof(get(cDbfs).c_str()); }
            double mer = atof(get(cMer).c_str());
            double ber = atof(get(cBer).c_str());
            switch (metric) {
            case METRIC_LEVEL: p.v = level; break;
            case METRIC_MER: p.v = mer; break;
            case METRIC_BER: p.v = ber; break;
            }
            char lb[256];
            snprintf(lb, sizeof(lb), "%s | Level %.1f %s | MER %.1f dB | BER %s | MCC %s MNC %s CC %s%s",
                     get(cTime).c_str(), level, dbm.empty() ? "dBFS" : "dBm", mer, get(cBer).c_str(),
                     get(cMcc).c_str(), get(cMnc).c_str(), get(cCc).c_str(), p.service ? "" : " | NO SERVICE");
            p.label = lb;
            pts.push_back(p);
        }
        if (pts.empty()) { return "No samples with GPS position in " + csvPath; }

        // Downsample very long drives so Google Earth / browsers stay responsive
        const size_t MAX_PTS = 20000;
        if (pts.size() > MAX_PTS) {
            std::vector<Pt> d;
            double step = (double)pts.size() / MAX_PTS;
            for (double i = 0; i < pts.size(); i += step) { d.push_back(pts[(size_t)i]); }
            pts.swap(d);
        }

        // Fixed scales so maps of different drives are comparable
        double lo, hi;
        std::string unit, name;
        bool logScale = false, invert = false;
        if (metric == METRIC_LEVEL) {
            if (useDbm) { lo = -115; hi = -65; unit = "dBm"; }
            else { lo = -95; hi = -45; unit = "dBFS"; }
            name = "Level";
        }
        else if (metric == METRIC_MER) { lo = 5; hi = 30; unit = "dB"; name = "MER"; }
        else { lo = 1e-1; hi = 1e-5; unit = ""; name = "BER"; logScale = true; invert = true; }

        auto norm = [&](double v) {
            double t;
            if (logScale) {
                double lv = std::log10((std::max)(v, 1e-7));
                t = (lv - std::log10(lo)) / (std::log10(hi) - std::log10(lo));
            }
            else {
                t = (v - lo) / (hi - lo);
            }
            (void)invert;
            return (std::min)(1.0, (std::max)(0.0, t));
        };

        std::string base = csvPath.size() > 4 ? csvPath.substr(0, csvPath.size() - 4) : csvPath;
        std::string suffix = (metric == METRIC_LEVEL ? "_level" : metric == METRIC_MER ? "_mer" : "_ber");
        std::string kmlPath = base + suffix + ".kml";
        std::string htmlPath = base + suffix + "_map.html";

        if (!writeKml(kmlPath, pts, norm, name, unit, lo, hi)) { return "Cannot write " + kmlPath; }
        if (!writeHtml(htmlPath, pts, norm, name, unit, lo, hi, logScale)) { return "Cannot write " + htmlPath; }
        size_t noSvc = std::count_if(pts.begin(), pts.end(), [](const Pt& p) { return !p.service; });
        return "Exported " + std::to_string(pts.size()) + " points (" + std::to_string(noSvc) + " without service): " +
               kmlPath + " and " + htmlPath;
    }

private:
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

    // red -> yellow -> green
    static void ramp(double t, int& r, int& g, int& b) {
        if (t < 0.5) { r = 230; g = (int)(40 + 400 * t); b = 40; }
        else { r = (int)(230 - 400 * (t - 0.5)); g = 230; b = 40; }
        r = (std::max)(0, (std::min)(255, r));
        g = (std::max)(0, (std::min)(255, g));
    }

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
    bool writeKml(const std::string& path, const std::vector<P>& pts, N norm, const std::string& name,
                  const std::string& unit, double lo, double hi) {
        FILE* f = fopen(path.c_str(), "w");
        if (!f) { return false; }
        fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n");
        fprintf(f, "<name>TETRA coverage - %s</name>\n", xmlEscape(name).c_str());
        fprintf(f, "<description>%s scale: red = %g %s, green = %g %s, black = no service</description>\n",
                xmlEscape(name).c_str(), lo, unit.c_str(), hi, unit.c_str());
        const int BINS = 10;
        for (int i = 0; i <= BINS; i++) {
            int r, g, b;
            ramp((double)i / BINS, r, g, b);
            // KML colors are aabbggrr
            fprintf(f, "<Style id=\"s%d\"><IconStyle><color>ff%02x%02x%02x</color><scale>0.5</scale>"
                       "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href></Icon></IconStyle></Style>\n",
                    i, b, g, r);
        }
        fprintf(f, "<Style id=\"nosvc\"><IconStyle><color>ff000000</color><scale>0.5</scale>"
                   "<Icon><href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href></Icon></IconStyle></Style>\n");
        for (auto& p : pts) {
            std::string st = p.service ? ("s" + std::to_string((int)std::lround(norm(p.v) * BINS))) : "nosvc";
            fprintf(f, "<Placemark><description>%s</description><styleUrl>#%s</styleUrl><Point><coordinates>%.6f,%.6f,0</coordinates></Point></Placemark>\n",
                    xmlEscape(p.label).c_str(), st.c_str(), p.lon, p.lat);
        }
        fprintf(f, "</Document>\n</kml>\n");
        fclose(f);
        return true;
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
