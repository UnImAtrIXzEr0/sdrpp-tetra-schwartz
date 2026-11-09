#pragma once
// Saved maps for drive tests without network: the tiles of an area around a
// centre (GPS fix, map centre or coordinates; radius or width x height), all
// zoom levels from an overview up to the chosen detail, from a provider whose
// terms allow saving (TopPlusOpen, basemap.de, Sentinel-2 - not
// OpenStreetMap). One folder per map, <root>/maps/<name>/, with the tiles as
// z/x/y files and map.json (provider, attribution with the date of saving as
// TopPlusOpen requires, area, zoom levels). Downloads one tile after the
// other over one connection with the edition's user agent.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <json.hpp>
#include "iso_time.h"
#include "map_tiles.h"

namespace mapstore {
    constexpr double M_PER_DEG = 6371008.8 * tiles::PI / 180.0;
    constexpr size_t MAX_TILES = 30000;          // about 600 MB
    constexpr double AVG_TILE_BYTES = 20000.0;   // estimate for the dialog
    constexpr int ZOOM_LEVELS = 7;               // overview levels below the detail level

    struct Area {
        double lat = 0, lon = 0;
        bool radius = true;
        double rKm = 10, wKm = 20, hKm = 20;
    };

    inline void bbox(const Area& a, double& la0, double& lo0, double& la1, double& lo1) {
        double hx = (a.radius ? a.rKm : a.wKm * 0.5) * 1000.0, hy = (a.radius ? a.rKm : a.hKm * 0.5) * 1000.0;
        double dla = hy / M_PER_DEG, dlo = hx / (M_PER_DEG * (std::max)(0.01, std::cos(a.lat * tiles::PI / 180)));
        la0 = std::clamp(a.lat - dla, -85.0, 85.0);
        la1 = std::clamp(a.lat + dla, -85.0, 85.0);
        lo0 = std::clamp(a.lon - dlo, -180.0, 179.9999);
        lo1 = std::clamp(a.lon + dlo, -180.0, 179.9999);
    }

    struct Range {
        int z, x0, x1, y0, y1;
        size_t count() const { return (size_t)(x1 - x0 + 1) * (size_t)(y1 - y0 + 1); }
    };

    inline int minZoomFor(int zMax) { return (std::max)(0, zMax - ZOOM_LEVELS + 1); }

    inline std::vector<Range> ranges(const Area& a, int zMax) {
        double la0, lo0, la1, lo1;
        bbox(a, la0, lo0, la1, lo1);
        std::vector<Range> r;
        for (int z = minZoomFor(zMax); z <= zMax; z++) {
            double xa, ya, xb, yb;
            tiles::toTile(la1, lo0, z, xa, ya);   // north-west
            tiles::toTile(la0, lo1, z, xb, yb);   // south-east
            int n = 1 << z;
            r.push_back({ z, std::clamp((int)std::floor(xa), 0, n - 1), std::clamp((int)std::floor(xb), 0, n - 1),
                          std::clamp((int)std::floor(ya), 0, n - 1), std::clamp((int)std::floor(yb), 0, n - 1) });
        }
        return r;
    }

    inline size_t countTiles(const Area& a, int zMax) {
        size_t n = 0;
        for (auto& r : ranges(a, zMax)) { n += r.count(); }
        return n;
    }

    // metres per tile pixel at zoom z
    inline double metersPerPixel(double lat, int z) { return 156543.03392 * std::cos(lat * tiles::PI / 180) / std::ldexp(1.0, z); }

    // folder name from a map name: letters, digits, '-', '_', '.', space -> '_'
    inline std::string folderName(const std::string& name) {
        std::string o;
        for (unsigned char c : name) {
            if (std::isalnum(c) || c == '-' || c == '_' || c == '.') { o += (char)c; }
            else if (c == ' ') { o += '_'; }
        }
        while (!o.empty() && o[0] == '.') { o.erase(0, 1); }
        return o.empty() ? "map" : o.substr(0, 60);
    }

    inline std::string extOf(const std::string& url) {
        size_t d = url.find_last_of('.');
        std::string e = d == std::string::npos ? ".img" : url.substr(d);
        return e.size() > 5 ? ".img" : e;
    }

    struct SavedMap {
        std::string dir, name, provider, attribution, ext, created;
        int zMin = 0, zMax = 0;
        double la0 = 0, lo0 = 0, la1 = 0, lo1 = 0;
        size_t tiles = 0, failed = 0;
        uint64_t bytes = 0;
    };

    inline std::vector<SavedMap> list(const std::string& root) {
        std::vector<SavedMap> out;
        std::error_code ec;
        for (auto it = std::filesystem::directory_iterator(root, ec); !ec && it != std::filesystem::directory_iterator(); it.increment(ec)) {
            std::string mj = (it->path() / "map.json").string();
            std::ifstream f(mj);
            if (!f) { continue; }
            try {
                nlohmann::json j = nlohmann::json::parse(f);
                SavedMap m;
                m.dir = it->path().string();
                m.name = j.value("name", it->path().filename().string());
                m.provider = j.value("provider", "");
                m.attribution = j.value("attribution", "");
                m.ext = j.value("ext", ".png");
                m.created = j.value("created", "");
                m.zMin = j.value("zMin", 0);
                m.zMax = j.value("zMax", 0);
                m.la0 = j.value("south", 0.0);
                m.la1 = j.value("north", 0.0);
                m.lo0 = j.value("west", 0.0);
                m.lo1 = j.value("east", 0.0);
                m.tiles = j.value("tiles", (size_t)0);
                m.failed = j.value("failed", (size_t)0);
                m.bytes = j.value("bytes", (uint64_t)0);
                out.push_back(m);
            }
            catch (...) {}
        }
        std::sort(out.begin(), out.end(), [](const SavedMap& a, const SavedMap& b) { return a.name < b.name; });
        return out;
    }

    inline tiles::Provider asProvider(const SavedMap& m) {
        tiles::Provider p;
        p.key = "saved:" + std::filesystem::path(m.dir).filename().string();
        p.name = m.name;
        p.url = m.dir + "/{z}/{x}/{y}" + m.ext;
        p.minZoom = m.zMin;
        p.maxZoom = m.zMax;
        p.attribution = m.attribution;
        p.local = true;
        return p;
    }

    inline bool removeMap(const SavedMap& m) {
        std::error_code ec;
        // only folders that are saved maps
        if (!std::filesystem::exists(std::filesystem::path(m.dir) / "map.json", ec)) { return false; }
        std::filesystem::remove_all(m.dir, ec);
        return !ec;
    }
}

class MapBuilder {
public:
    ~MapBuilder() {
        cancelReq = true;
        if (worker.joinable()) { worker.join(); }
    }

    bool busy() const { return running.load(); }
    void cancel() { cancelReq = true; }
    float progress() const { return total ? (float)done.load() / (float)total : 0.0f; }
    // after status() == "saved"
    size_t savedTiles = 0, failedTiles = 0;

    std::string status() {
        std::lock_guard<std::mutex> lck(mtx);
        return statusText;
    }

    // true when the download started; error text in status() otherwise
    bool start(const std::string& root, const std::string& name, const tiles::Provider& prov, const mapstore::Area& area, int zMax, const std::string& ua) {
        if (running) { return false; }
        if (worker.joinable()) { worker.join(); }
        auto rs = mapstore::ranges(area, zMax);
        size_t n = 0;
        for (auto& r : rs) { n += r.count(); }
        if (!prov.storable) { setStatus("This map may not be saved (terms of use)."); return false; }
        if (n > mapstore::MAX_TILES) { setStatus("Area too large for this detail level."); return false; }
        std::string dir = root + "/" + mapstore::folderName(name);
        std::error_code ec;
        if (std::filesystem::exists(dir, ec)) { setStatus("A saved map with this name exists already."); return false; }
        if (!std::filesystem::create_directories(dir, ec) || ec) { setStatus("Cannot create " + dir); return false; }
        total = n;
        done = 0;
        cancelReq = false;
        running = true;
        setStatus("");
        worker = std::thread([this, dir, name, prov, area, rs, zMax, ua]() { run(dir, name, prov, area, rs, zMax, ua); });
        return true;
    }

private:
    std::thread worker;
    std::atomic<bool> running { false }, cancelReq { false };
    std::atomic<size_t> done { 0 };
    size_t total = 0;
    std::mutex mtx;
    std::string statusText;

    void setStatus(const std::string& s) {
        std::lock_guard<std::mutex> lck(mtx);
        statusText = s;
    }

    void run(std::string dir, std::string name, tiles::Provider prov, mapstore::Area area, std::vector<mapstore::Range> rs, int zMax, std::string ua) {
        std::string ext = mapstore::extOf(prov.url);
        std::vector<uint64_t> all;
        for (auto& r : rs) {
            for (int x = r.x0; x <= r.x1; x++) { for (int y = r.y0; y <= r.y1; y++) { all.push_back(tiles::key(r.z, x, y)); } }
        }
        auto pathOf = [&](uint64_t k) {
            return dir + "/" + std::to_string(tiles::keyZ(k)) + "/" + std::to_string(tiles::keyX(k)) + "/" + std::to_string(tiles::keyY(k)) + ext;
        };
        std::vector<uint64_t> failed;
        uint64_t bytes = 0;
        // two passes: the second one retries what failed in the first
        for (int pass = 0; pass < 2 && !cancelReq; pass++) {
            std::vector<uint64_t> list = pass == 0 ? all : failed;
            failed.clear();
            for (size_t i = 0; i < list.size() && !cancelReq; i += 8) {
                std::vector<std::pair<std::string, std::string>> jobs;
                std::vector<uint64_t> keys;
                for (size_t j = i; j < (std::min)(list.size(), i + 8); j++) {
                    uint64_t k = list[j];
                    std::string p = pathOf(k);
                    std::error_code ec;
                    std::filesystem::create_directories(std::filesystem::path(p).parent_path(), ec);
                    jobs.push_back({ tiles::expand(prov.url, tiles::keyZ(k), tiles::keyX(k), tiles::keyY(k)), p + ".part" });
                    keys.push_back(k);
                }
                tiles::curlFetch(jobs, ua);
                for (size_t j = 0; j < jobs.size(); j++) {
                    std::error_code ec;
                    uint64_t sz = std::filesystem::exists(jobs[j].second, ec) ? (uint64_t)std::filesystem::file_size(jobs[j].second, ec) : 0;
                    if (sz > 0) {
                        std::filesystem::rename(jobs[j].second, jobs[j].second.substr(0, jobs[j].second.size() - 5), ec);
                        bytes += sz;
                        if (pass == 0) { done++; }
                    }
                    else {
                        std::filesystem::remove(jobs[j].second, ec);
                        failed.push_back(keys[j]);
                        if (pass == 0) { done++; }
                    }
                }
            }
        }
        if (cancelReq) {
            std::error_code ec;
            std::filesystem::remove_all(dir, ec);
            setStatus("Cancelled.");
            running = false;
            return;
        }
        double la0, lo0, la1, lo1;
        mapstore::bbox(area, la0, lo0, la1, lo1);
        std::string date = isoLocalTime(time(nullptr)).substr(0, 10);
        nlohmann::json j;
        j["name"] = name;
        j["provider"] = prov.name;
        j["providerKey"] = prov.key;
        // TopPlusOpen: saved data names the date of saving
        j["attribution"] = prov.attribution + " - saved " + date;
        j["ext"] = ext;
        j["created"] = isoLocalTime(time(nullptr));
        j["zMin"] = mapstore::minZoomFor(zMax);
        j["zMax"] = zMax;
        j["south"] = la0;
        j["north"] = la1;
        j["west"] = lo0;
        j["east"] = lo1;
        j["centerLat"] = area.lat;
        j["centerLon"] = area.lon;
        j["tiles"] = all.size() - failed.size();
        j["failed"] = failed.size();
        j["bytes"] = bytes;
        std::ofstream f(dir + "/map.json");
        f << j.dump(2);
        f.close();
        savedTiles = all.size() - failed.size();
        failedTiles = failed.size();
        setStatus("saved");
        running = false;
    }
};
