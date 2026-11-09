#pragma once
// Map tiles under the drive map (Web Mercator z/x/y). Only the tiles in view
// are fetched (no prefetching or bulk download, as the OSM tile usage policy
// requires), over one connection, with the edition's user agent. Tiles are
// kept in a disk cache (re-fetched after 30 days, at most 500 MB, oldest
// removed first). A worker thread fetches and decodes; the GUI thread turns
// at most a few tiles per frame into textures. While a tile is missing, the
// nearest coarser tile that is loaded is drawn in its place.
// Contains the stb_image implementation (static): include in one
// translation unit only.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <imgui.h>
#include <gui/widgets/texture.h>
#ifdef _WIN32
#include <windows.h>
#endif

#define STB_IMAGE_STATIC
#define STBI_WINDOWS_UTF8   // file names are UTF-8
#define STB_IMAGE_IMPLEMENTATION
#include <imgui/stb_image.h>

namespace tiles {
    constexpr double PI = 3.14159265358979323846;

    struct Provider {
        std::string key;          // cache folder
        std::string name;         // shown in the menu
        std::string url;          // {z} {x} {y}
        int maxZoom = 18;
        std::string attribution;  // shown on the map, always visible
    };

    // year: for the attribution of the BKG services ("year of the last data retrieval")
    inline std::vector<Provider> providers(int year) {
        std::string y = std::to_string(year);
        return {
            { "osm", "OpenStreetMap", "https://tile.openstreetmap.org/{z}/{x}/{y}.png", 19, "\xC2\xA9 OpenStreetMap contributors" },
            { "topplusopen", "TopPlusOpen", "https://sgx.geodatenzentrum.de/wmts_topplus_open/tile/1.0.0/web/default/WEBMERCATOR/{z}/{y}/{x}.png", 18,
              "\xC2\xA9 BKG (" + y + ") dl-de/by-2-0, data sources: sg.geodatenzentrum.de/web_public/Datenquellen_TopPlus_Open.pdf" },
            { "basemapde", "basemap.de", "https://sgx.geodatenzentrum.de/wmts_basemapde/tile/1.0.0/de_basemapde_web_raster_farben/default/GLOBAL_WEBMERCATOR/{z}/{y}/{x}.png", 18,
              "\xC2\xA9 GeoBasis-DE / BKG (" + y + ") CC BY 4.0" },
            { "s2cloudless2024", "Satellite (Sentinel-2)", "https://tiles.maps.eox.at/wmts/1.0.0/s2cloudless-2024_3857/default/g/{z}/{y}/{x}.jpg", 14,
              "Sentinel-2 cloudless - https://s2maps.eu by EOX IT Services GmbH (Contains modified Copernicus Sentinel data 2024)" },
        };
    }

    // fractional tile coordinates at zoom z
    inline void toTile(double lat, double lon, int z, double& x, double& y) {
        double n = std::ldexp(1.0, z);
        lat = std::clamp(lat, -85.05112878, 85.05112878);
        double r = lat * PI / 180;
        x = (lon + 180.0) / 360.0 * n;
        y = (1.0 - std::log(std::tan(r) + 1.0 / std::cos(r)) / PI) / 2.0 * n;
    }

    inline void fromTile(double x, double y, int z, double& lat, double& lon) {
        double n = std::ldexp(1.0, z);
        lon = x / n * 360.0 - 180.0;
        lat = std::atan(std::sinh(PI * (1.0 - 2.0 * y / n))) * 180.0 / PI;
    }

    // zoom whose tile pixels are closest to 'mpp' metres per screen pixel
    inline int zoomFor(double lat, double mpp, int maxZoom) {
        double z = std::log2(156543.03392 * std::cos(lat * PI / 180) / (std::max)(mpp, 1e-3));
        return std::clamp((int)std::lround(z), 0, maxZoom);
    }

    inline std::string expand(const std::string& tpl, int z, int x, int y) {
        std::string o;
        for (size_t i = 0; i < tpl.size(); i++) {
            if (tpl.compare(i, 3, "{z}") == 0) { o += std::to_string(z); i += 2; }
            else if (tpl.compare(i, 3, "{x}") == 0) { o += std::to_string(x); i += 2; }
            else if (tpl.compare(i, 3, "{y}") == 0) { o += std::to_string(y); i += 2; }
            else { o += tpl[i]; }
        }
        return o;
    }

    inline uint64_t key(int z, int x, int y) { return ((uint64_t)z << 56) | ((uint64_t)(uint32_t)x << 28) | (uint64_t)(uint32_t)y; }
    inline int keyZ(uint64_t k) { return (int)(k >> 56); }
    inline int keyX(uint64_t k) { return (int)((k >> 28) & 0x0FFFFFFF); }
    inline int keyY(uint64_t k) { return (int)(k & 0x0FFFFFFF); }
}

class TileLayer {
public:
    static constexpr int CACHE_DAYS = 30;
    static constexpr uint64_t CACHE_MAX_BYTES = 500ull * 1024 * 1024;
    static constexpr size_t MAX_TEXTURES = 256;

    ~TileLayer() {
        {
            std::lock_guard<std::mutex> lck(mtx);
            stopReq = true;
        }
        cv.notify_all();
        if (worker.joinable()) { worker.join(); }
        for (auto& t : texs) { texture::destroy(t.second.id); }
    }

    void setCacheDir(const std::string& dir) {
        std::lock_guard<std::mutex> lck(mtx);
        cacheDir = dir;
    }
    void setUserAgent(const std::string& ua) {
        std::lock_guard<std::mutex> lck(mtx);
        userAgent = ua;
    }

    // provider: nullptr = no map. GUI thread.
    void setProvider(const tiles::Provider* p) {
        std::string k = p ? p->key + "|" + p->url : std::string();
        if (k == provKey) { return; }
        provKey = k;
        for (auto& t : texs) { texture::destroy(t.second.id); }
        texs.clear();
        {
            std::lock_guard<std::mutex> lck(mtx);
            prov = p ? *p : tiles::Provider();
            active = (p != nullptr);
            wanted.clear();
            ready.clear();
            failed.clear();
            gen++;
        }
        lastWanted.clear();
        if (p && !worker.joinable()) { worker = std::thread([this]() { workerLoop(); }); }
    }

    bool isActive() const { return !provKey.empty(); }

    // Draws the tiles of the view. toScreen: geographic position -> screen position.
    // GUI thread.
    void draw(ImDrawList* dl, ImVec2 p0, ImVec2 p1, double cLat, double mpp,
              const std::function<ImVec2(double, double)>& toScreen,
              const std::function<void(double, double, double&, double&)>& toGeo, float alpha) {
        if (provKey.empty()) { return; }
        frame++;
        uploadReady();
        int maxZ;
        {
            std::lock_guard<std::mutex> lck(mtx);
            maxZ = prov.maxZoom;
        }
        int z = tiles::zoomFor(cLat, mpp, maxZ);
        // tile range of the view (from the four corners)
        double x0 = 1e18, x1 = -1e18, y0 = 1e18, y1 = -1e18;
        ImVec2 corners[4] = { p0, ImVec2(p1.x, p0.y), ImVec2(p0.x, p1.y), p1 };
        for (auto& c : corners) {
            double la, lo, tx, ty;
            toGeo(c.x, c.y, la, lo);
            tiles::toTile(la, lo, z, tx, ty);
            x0 = (std::min)(x0, tx); x1 = (std::max)(x1, tx);
            y0 = (std::min)(y0, ty); y1 = (std::max)(y1, ty);
        }
        int n = 1 << z;
        int tx0 = (std::max)(0, (int)std::floor(x0)), tx1 = (std::min)(n - 1, (int)std::floor(x1));
        int ty0 = (std::max)(0, (int)std::floor(y0)), ty1 = (std::min)(n - 1, (int)std::floor(y1));
        if ((tx1 - tx0 + 1) * (ty1 - ty0 + 1) > 120) { return; }   // not a sensible view for tiles

        // wanted: the tiles of the view, nearest to the centre first
        std::vector<uint64_t> want;
        double cx = (x0 + x1) * 0.5, cy = (y0 + y1) * 0.5;
        for (int ty = ty0; ty <= ty1; ty++) { for (int tx = tx0; tx <= tx1; tx++) { want.push_back(tiles::key(z, tx, ty)); } }
        std::sort(want.begin(), want.end(), [&](uint64_t a, uint64_t b) {
            double da = std::hypot(tiles::keyX(a) + 0.5 - cx, tiles::keyY(a) + 0.5 - cy);
            double db = std::hypot(tiles::keyX(b) + 0.5 - cx, tiles::keyY(b) + 0.5 - cy);
            return da < db;
        });
        std::vector<uint64_t> missing;
        for (uint64_t k : want) { if (!texs.count(k)) { missing.push_back(k); } }
        if (missing != lastWanted) {
            lastWanted = missing;
            {
                std::lock_guard<std::mutex> lck(mtx);
                wanted = missing;
            }
            cv.notify_all();
        }

        ImU32 col = IM_COL32(255, 255, 255, (int)std::lround(std::clamp(alpha, 0.0f, 1.0f) * 255));
        for (uint64_t k : want) {
            int tz = z, tx = tiles::keyX(k), ty = tiles::keyY(k);
            // the tile itself, or the part of the nearest coarser tile that is loaded
            ImTextureID tex = nullptr;
            ImVec2 uv0(0, 0), uv1(1, 1);
            for (int up = 0; up <= 6 && up <= tz; up++) {
                auto it = texs.find(tiles::key(tz - up, tx >> up, ty >> up));
                if (it == texs.end()) { continue; }
                it->second.lastUse = frame;
                tex = it->second.id;
                float s = 1.0f / (float)(1 << up);
                uv0 = ImVec2((tx - ((tx >> up) << up)) * s, (ty - ((ty >> up) << up)) * s);
                uv1 = ImVec2(uv0.x + s, uv0.y + s);
                break;
            }
            if (!tex) { continue; }
            double la, lo;
            ImVec2 q[4];
            const int dx[4] = { 0, 1, 1, 0 }, dy[4] = { 0, 0, 1, 1 };
            for (int i = 0; i < 4; i++) {
                tiles::fromTile(tx + dx[i], ty + dy[i], tz, la, lo);
                q[i] = toScreen(la, lo);
            }
            dl->AddImageQuad(tex, q[0], q[1], q[2], q[3], uv0, ImVec2(uv1.x, uv0.y), uv1, ImVec2(uv0.x, uv1.y), col);
        }
        evictTextures();
    }

    // "", "loading", "offline"
    std::string status() {
        std::lock_guard<std::mutex> lck(mtx);
        if (!active) { return ""; }
        if (offline) { return "offline"; }
        return wanted.empty() ? "" : "loading";
    }

    int textureCount() const { return (int)texs.size(); }

private:
    struct Tex {
        ImTextureID id;
        uint64_t lastUse;
    };
    struct Ready {
        uint64_t key;
        int w, h;
        std::vector<uint8_t> rgba;
        uint64_t gen;
    };

    // GUI thread only
    std::string provKey;
    std::unordered_map<uint64_t, Tex> texs;
    std::vector<uint64_t> lastWanted;
    uint64_t frame = 0;

    // shared with the worker
    std::mutex mtx;
    std::condition_variable cv;
    tiles::Provider prov;
    bool active = false;
    std::string cacheDir, userAgent = "SDRPP-AIA";
    std::vector<uint64_t> wanted;
    std::deque<Ready> ready;
    std::unordered_map<uint64_t, std::pair<double, int>> failed;   // key -> (time, count)
    bool offline = false, stopReq = false;
    uint64_t gen = 0;
    std::thread worker;
    int downloads = 0;

    void uploadReady() {
        for (int i = 0; i < 4; i++) {
            Ready r;
            {
                std::lock_guard<std::mutex> lck(mtx);
                if (ready.empty()) { return; }
                r = std::move(ready.front());
                ready.pop_front();
                if (r.gen != gen) { continue; }
            }
            if (texs.count(r.key)) { continue; }
            texs[r.key] = { texture::create(r.rgba.data(), r.w, r.h), frame };
        }
    }

    void evictTextures() {
        if (texs.size() <= MAX_TEXTURES) { return; }
        std::vector<std::pair<uint64_t, uint64_t>> v;
        for (auto& t : texs) { v.push_back({ t.second.lastUse, t.first }); }
        std::sort(v.begin(), v.end());
        for (size_t i = 0; i < v.size() - MAX_TEXTURES; i++) {
            if (v[i].first == frame) { break; }   // in use this frame
            texture::destroy(texs[v[i].second].id);
            texs.erase(v[i].second);
        }
    }

    static double nowS() {
        return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    std::string cachePath(const tiles::Provider& p, uint64_t k) const {
        std::string ext = p.url.substr(p.url.find_last_of('.') == std::string::npos ? p.url.size() : p.url.find_last_of('.'));
        if (ext.size() > 5) { ext = ".img"; }
        return cacheDir + "/" + p.key + "/" + std::to_string(tiles::keyZ(k)) + "/" + std::to_string(tiles::keyX(k)) + "/" +
               std::to_string(tiles::keyY(k)) + ext;
    }

    static bool fresh(const std::string& path) {
        std::error_code ec;
        auto t = std::filesystem::last_write_time(path, ec);
        if (ec) { return false; }
        auto age = std::filesystem::file_time_type::clock::now() - t;
        return age < std::chrono::hours(24 * CACHE_DAYS);
    }

    static bool decode(const std::string& path, Ready& r) {
        int w, h, n;
        stbi_uc* d = stbi_load(path.c_str(), &w, &h, &n, 4);
        if (!d) { return false; }
        r.w = w;
        r.h = h;
        r.rgba.assign(d, d + (size_t)w * h * 4);
        stbi_image_free(d);
        return true;
    }

    // one curl run for up to 2 tiles, one after the other over one connection
    // (short: the destructor waits for it)
    bool curlBatch(const std::vector<std::pair<std::string, std::string>>& jobs, const std::string& ua) {
#ifdef _WIN32
        std::string cmd = "curl.exe -fsSL --max-time 15 -A \"" + ua + "\"";
        for (auto& j : jobs) { cmd += " -o \"" + j.second + "\" \"" + j.first + "\""; }
        STARTUPINFOA si;
        PROCESS_INFORMATION pi;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        ZeroMemory(&pi, sizeof(pi));
        std::vector<char> buf(cmd.begin(), cmd.end());
        buf.push_back('\0');
        int rc = -1;
        if (CreateProcessA(NULL, buf.data(), NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) {
            WaitForSingleObject(pi.hProcess, 60000);
            DWORD code = 1;
            GetExitCodeProcess(pi.hProcess, &code);
            rc = (int)code;
            CloseHandle(pi.hProcess);
            CloseHandle(pi.hThread);
        }
        return rc == 0;
#else
        auto q = [](const std::string& in) {
            std::string o = "'";
            for (char c : in) { if (c == '\'') { o += "'\\''"; } else { o += c; } }
            return o + "'";
        };
        // env -u: don't let curl pick up the AppImage's bundled libraries
        std::string cmd = "env -u LD_LIBRARY_PATH curl -fsSL --max-time 15 -A " + q(ua);
        for (auto& j : jobs) { cmd += " -o " + q(j.second) + " " + q(j.first); }
        cmd += " >/dev/null 2>&1";
        return std::system(cmd.c_str()) == 0;
#endif
    }

    // keep the cache below its size limit: remove the oldest tiles
    void pruneCache(const std::string& dir) {
        std::error_code ec;
        std::vector<std::pair<std::filesystem::file_time_type, std::pair<std::string, uint64_t>>> files;
        uint64_t total = 0;
        for (auto it = std::filesystem::recursive_directory_iterator(dir, ec); !ec && it != std::filesystem::recursive_directory_iterator(); it.increment(ec)) {
            if (!it->is_regular_file(ec)) { continue; }
            uint64_t sz = (uint64_t)it->file_size(ec);
            total += sz;
            files.push_back({ it->last_write_time(ec), { it->path().string(), sz } });
        }
        if (total <= CACHE_MAX_BYTES) { return; }
        std::sort(files.begin(), files.end(), [](auto& a, auto& b) { return a.first < b.first; });
        for (auto& f : files) {
            if (total <= CACHE_MAX_BYTES * 8 / 10) { break; }
            std::filesystem::remove(f.second.first, ec);
            total -= f.second.second;
        }
    }

    void workerLoop() {
        bool pruned = false;
        while (true) {
            tiles::Provider p;
            std::string dir, ua;
            std::vector<uint64_t> todo;
            uint64_t g;
            {
                std::unique_lock<std::mutex> lck(mtx);
                cv.wait_for(lck, std::chrono::seconds(5), [&]() { return stopReq || (active && !wanted.empty()); });
                if (stopReq) { return; }
                if (!active || wanted.empty()) { continue; }
                p = prov;
                dir = cacheDir;
                ua = userAgent;
                g = gen;
                double t = nowS();
                for (uint64_t k : wanted) {
                    auto f = failed.find(k);
                    // after a failure wait 60 s, doubling up to 1 h
                    if (f != failed.end() && t - f->second.first < (std::min)(3600.0, 60.0 * std::ldexp(1.0, (std::min)(f->second.second - 1, 6)))) { continue; }
                    todo.push_back(k);
                    if (todo.size() >= 2) { break; }
                }
                if (todo.empty()) {
                    // everything wanted failed recently: wait for a change
                    cv.wait_for(lck, std::chrono::seconds(5), [&]() { return stopReq || gen != g; });
                    continue;
                }
            }
            if (!pruned && !dir.empty()) { pruneCache(dir); pruned = true; }

            // cache first, then one download batch for the rest
            std::vector<std::pair<std::string, std::string>> jobs;
            std::vector<uint64_t> jobKeys;
            std::vector<Ready> got;
            for (uint64_t k : todo) {
                std::string path = cachePath(p, k);
                Ready r;
                r.key = k;
                r.gen = g;
                if (fresh(path) && decode(path, r)) { got.push_back(std::move(r)); continue; }
                std::error_code ec;
                std::filesystem::create_directories(std::filesystem::path(path).parent_path(), ec);
                jobs.push_back({ tiles::expand(p.url, tiles::keyZ(k), tiles::keyX(k), tiles::keyY(k)), path + ".part" });
                jobKeys.push_back(k);
            }
            int ok = 0, bad = 0;
            if (!jobs.empty()) {
                curlBatch(jobs, ua);
                for (size_t i = 0; i < jobs.size(); i++) {
                    std::string part = jobs[i].second, path = part.substr(0, part.size() - 5);
                    Ready r;
                    r.key = jobKeys[i];
                    r.gen = g;
                    std::error_code ec;
                    if (std::filesystem::exists(part, ec) && decode(part, r)) {
                        std::filesystem::rename(part, path, ec);
                        got.push_back(std::move(r));
                        ok++;
                    }
                    else {
                        std::filesystem::remove(part, ec);
                        // an outdated tile in the cache is still better than none
                        if (std::filesystem::exists(path, ec) && decode(path, r)) { got.push_back(std::move(r)); }
                        bad++;
                        std::lock_guard<std::mutex> lck(mtx);
                        auto& f = failed[jobKeys[i]];
                        f.first = nowS();
                        f.second++;
                    }
                }
                downloads += ok;
                if (downloads >= 500) { downloads = 0; pruneCache(dir); }
            }
            {
                std::lock_guard<std::mutex> lck(mtx);
                if (!jobs.empty()) { offline = (ok == 0 && bad > 0); }
                for (auto& r : got) {
                    wanted.erase(std::remove(wanted.begin(), wanted.end(), r.key), wanted.end());
                    ready.push_back(std::move(r));
                }
            }
        }
    }
};
