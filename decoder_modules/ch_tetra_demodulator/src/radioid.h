#pragma once
// Offline lookup of DMR IDs (= ISSI in amateur TETRA networks) from the
// radioid.net database dumps. 7 digits: users (user.csv), 6 digits:
// repeaters/automated stations (rptrs.json). Files are loaded in a
// background thread; the optional download uses the system's curl
// (Linux, Windows 10/11 ship it), so nothing is installed or changed.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <json.hpp>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

struct RadioIdEntry {
    std::string callsign;
    std::string info; // name, city, country or repeater location/frequency
    bool repeater = false;
};

class RadioIdDb {
public:
    static constexpr const char* USERS_URL = "https://radioid.net/static/user.csv";
    static constexpr const char* RPTRS_URL = "https://radioid.net/static/rptrs.json";
    static constexpr const char* BM_TG_URL = "https://api.brandmeister.network/v2/talkgroup";

    ~RadioIdDb() {
        if (worker.joinable()) { worker.join(); }
    }

    void setDirectory(const std::string& dir) {
        std::lock_guard<std::mutex> lck(mtx);
        directory = dir;
    }

    std::string usersPath() { return dirCopy() + "/radioid_user.csv"; }
    std::string rptrsPath() { return dirCopy() + "/radioid_rptrs.json"; }
    std::string tgPath() { return dirCopy() + "/brandmeister_talkgroups.json"; }

    size_t talkgroupCount() { return nTg.load(); }

    // Brandmeister talkgroup name for a group SSI
    bool lookupGroup(uint32_t id, std::string& name) {
        std::lock_guard<std::mutex> lck(mtx);
        auto it = tgNames.find(id);
        if (it == tgNames.end()) { return false; }
        name = it->second;
        return true;
    }

    bool busy() { return isBusy.load(); }

    std::string status() {
        std::lock_guard<std::mutex> lck(mtx);
        return statusText;
    }

    size_t userCount() { return nUsers.load(); }
    size_t repeaterCount() { return nRptrs.load(); }

    bool lookup(uint32_t id, RadioIdEntry& out) {
        std::lock_guard<std::mutex> lck(mtx);
        auto it = db.find(id);
        if (it == db.end()) { return false; }
        out = it->second;
        return true;
    }

    // Load whatever files exist (non-blocking)
    void loadAsync() { runAsync(false); }

    // Download both dumps with curl, then load (non-blocking)
    void downloadAsync() { runAsync(true); }

private:
    std::string dirCopy() {
        std::lock_guard<std::mutex> lck(mtx);
        return directory;
    }

    void setStatus(const std::string& s) {
        std::lock_guard<std::mutex> lck(mtx);
        statusText = s;
    }

    void runAsync(bool download) {
        if (isBusy.exchange(true)) { return; }
        if (worker.joinable()) { worker.join(); }
        worker = std::thread([this, download]() {
            if (download) { doDownload(); }
            doLoad();
            isBusy = false;
        });
    }

    static bool curlFetch(const std::string& url, const std::string& dest) {
        std::string tmp = dest + ".part";
        std::error_code ec;
        std::filesystem::remove(tmp, ec);
#ifdef _WIN32
        // CreateProcess with CREATE_NO_WINDOW: no console window pops up
        std::string cmd = "curl.exe -fsSL --max-time 300 -A SDRPP-TETRA -o \"" + tmp + "\" \"" + url + "\"";
        STARTUPINFOA si;
        PROCESS_INFORMATION pi;
        ZeroMemory(&si, sizeof(si));
        si.cb = sizeof(si);
        ZeroMemory(&pi, sizeof(pi));
        std::vector<char> cmdBuf(cmd.begin(), cmd.end());
        cmdBuf.push_back('\0');
        int rc = -1;
        if (CreateProcessA(NULL, cmdBuf.data(), NULL, NULL, FALSE, CREATE_NO_WINDOW, NULL, NULL, &si, &pi)) {
            WaitForSingleObject(pi.hProcess, 330000);
            DWORD code = 1;
            GetExitCodeProcess(pi.hProcess, &code);
            rc = (int)code;
            CloseHandle(pi.hProcess);
            CloseHandle(pi.hThread);
        }
#else
        auto q = [](const std::string& in) {
            std::string o = "'";
            for (char c : in) { if (c == '\'') { o += "'\\''"; } else { o += c; } }
            return o + "'";
        };
        // env -u: don't let curl pick up the AppImage's bundled libraries
        std::string cmd = "env -u LD_LIBRARY_PATH curl -fsSL --max-time 300 -A 'SDRPP-TETRA' -o " + q(tmp) + " " + q(url) + " >/dev/null 2>&1";
        int rc = std::system(cmd.c_str());
#endif
        if (rc != 0 || !std::filesystem::exists(tmp) || std::filesystem::file_size(tmp, ec) < 100) {
            std::filesystem::remove(tmp, ec);
            return false;
        }
        std::filesystem::rename(tmp, dest, ec);
        return !ec;
    }

    void doDownload() {
        setStatus("Downloading user list...");
        bool u = curlFetch(USERS_URL, usersPath());
        setStatus("Downloading repeater list...");
        bool r = curlFetch(RPTRS_URL, rptrsPath());
        setStatus("Downloading Brandmeister talkgroups...");
        bool t = curlFetch(BM_TG_URL, tgPath());
        tgDownloadFailed = !t;
        if (!u || !r || !t) {
            setStatus(std::string("Download failed (") + (!u ? "users " : "") + (!r ? "repeaters " : "") + (!t ? "talkgroups" : "") +
                      "). Download manually (radioid.net / api.brandmeister.network) into the folder.");
        }
    }

    static std::vector<std::string> splitCsv(const std::string& line) {
        std::vector<std::string> out;
        std::string cur;
        bool quoted = false;
        for (size_t i = 0; i < line.size(); i++) {
            char c = line[i];
            if (c == '"') {
                if (quoted && i + 1 < line.size() && line[i + 1] == '"') { cur += '"'; i++; }
                else { quoted = !quoted; }
            }
            else if (c == ',' && !quoted) { out.push_back(cur); cur.clear(); }
            else if (c != '\r') { cur += c; }
        }
        out.push_back(cur);
        return out;
    }

    void doLoad() {
        std::unordered_map<uint32_t, RadioIdEntry> fresh;
        size_t users = 0, rptrs = 0;
        std::string err;

        // Users: CSV with header (RADIO_ID,CALLSIGN,FIRST_NAME,LAST_NAME,CITY,STATE,COUNTRY,...)
        std::ifstream uf(usersPath());
        if (uf) {
            setStatus("Loading user list...");
            std::string line;
            int cId = -1, cCall = -1, cFirst = -1, cLast = -1, cCity = -1, cCountry = -1;
            if (std::getline(uf, line)) {
                auto h = splitCsv(line);
                for (int i = 0; i < (int)h.size(); i++) {
                    std::string k = h[i];
                    for (auto& ch : k) { ch = (char)toupper((unsigned char)ch); }
                    if (k == "RADIO_ID" || k == "ID") { cId = i; }
                    else if (k == "CALLSIGN") { cCall = i; }
                    else if (k == "FIRST_NAME") { cFirst = i; }
                    else if (k == "LAST_NAME") { cLast = i; }
                    else if (k == "CITY") { cCity = i; }
                    else if (k == "COUNTRY") { cCountry = i; }
                }
            }
            if (cId < 0 || cCall < 0) {
                err = "user.csv: unexpected header";
            }
            else {
                auto get = [](const std::vector<std::string>& f, int i) { return (i >= 0 && i < (int)f.size()) ? f[i] : std::string(); };
                while (std::getline(uf, line)) {
                    auto f = splitCsv(line);
                    char* end = nullptr;
                    std::string idStr = get(f, cId);
                    unsigned long id = strtoul(idStr.c_str(), &end, 10);
                    if (end == idStr.c_str() || id == 0 || id > 0xFFFFFF) { continue; }
                    RadioIdEntry e;
                    e.callsign = get(f, cCall);
                    std::string name = get(f, cFirst);
                    std::string last = get(f, cLast);
                    if (!last.empty()) { name += (name.empty() ? "" : " ") + last; }
                    std::string loc = get(f, cCity);
                    std::string country = get(f, cCountry);
                    if (!country.empty()) { loc += (loc.empty() ? "" : ", ") + country; }
                    e.info = name + (loc.empty() ? "" : " - " + loc);
                    fresh[(uint32_t)id] = std::move(e);
                    users++;
                }
            }
        }

        // Repeaters: JSON {"rptrs":[{"id":..,"callsign":..,"city":..,"frequency":..},...]}
        std::ifstream rf(rptrsPath());
        if (rf) {
            setStatus("Loading repeater list...");
            try {
                nlohmann::json j = nlohmann::json::parse(rf, nullptr, true, true);
                const nlohmann::json& arr = j.contains("rptrs") ? j["rptrs"] : j;
                for (auto& r : arr) {
                    if (!r.is_object()) { continue; }
                    uint32_t id = 0;
                    for (const char* key : { "id", "locator" }) {
                        if (!r.contains(key)) { continue; }
                        if (r[key].is_number_unsigned()) { id = r[key].get<uint32_t>(); }
                        else if (r[key].is_string()) { id = (uint32_t)strtoul(r[key].get<std::string>().c_str(), nullptr, 10); }
                        if (id) { break; }
                    }
                    if (!id || id > 0xFFFFFF) { continue; }
                    RadioIdEntry e;
                    e.repeater = true;
                    e.callsign = (r.contains("callsign") && r["callsign"].is_string()) ? r["callsign"].get<std::string>() : "";
                    std::string city = (r.contains("city") && r["city"].is_string()) ? r["city"].get<std::string>() : "";
                    std::string freq = (r.contains("frequency") && r["frequency"].is_string()) ? r["frequency"].get<std::string>() : "";
                    e.info = "Repeater" + (city.empty() ? "" : " " + city) + (freq.empty() ? "" : " " + freq + " MHz");
                    fresh[id] = std::move(e);
                    rptrs++;
                }
            }
            catch (const std::exception& ex) {
                err += std::string(err.empty() ? "" : "; ") + "rptrs.json: " + ex.what();
            }
        }

        // Brandmeister talkgroups: JSON object {"<id>": "<name>", ...}
        std::unordered_map<uint32_t, std::string> tgs;
        std::ifstream tf(tgPath());
        if (tf) {
            try {
                nlohmann::json j = nlohmann::json::parse(tf, nullptr, true, true);
                if (j.is_object()) {
                    for (auto it = j.begin(); it != j.end(); ++it) {
                        unsigned long id = strtoul(it.key().c_str(), nullptr, 10);
                        if (!id || id > 0xFFFFFF || !it.value().is_string()) { continue; }
                        tgs[(uint32_t)id] = it.value().get<std::string>();
                    }
                }
            }
            catch (const std::exception& ex) {
                err += std::string(err.empty() ? "" : "; ") + "talkgroups: " + ex.what();
            }
        }
        nTg = tgs.size();
        {
            std::lock_guard<std::mutex> lck(mtx);
            db.swap(fresh);
            tgNames.swap(tgs);
        }
        nUsers = users;
        nRptrs = rptrs;
        if (!err.empty()) { setStatus(err); }
        else if (!users && !rptrs && !nTg.load()) { setStatus("No radioid.net / Brandmeister files yet - press Download"); }
        else {
            std::string loaded = "Loaded " + std::to_string(users) + " users, " + std::to_string(rptrs) + " repeaters, " +
                                 std::to_string(nTg.load()) + " BM talkgroups";
            std::lock_guard<std::mutex> lck(mtx);
            if (statusText.rfind("Download failed", 0) == 0) { statusText = loaded + ". " + statusText; }
            else { statusText = loaded; }
        }
    }

    std::mutex mtx;
    std::string directory = ".";
    std::string statusText = "Not loaded";
    std::unordered_map<uint32_t, RadioIdEntry> db;
    std::atomic<size_t> nUsers { 0 }, nRptrs { 0 }, nTg { 0 };
    std::unordered_map<uint32_t, std::string> tgNames;
    std::atomic<bool> tgDownloadFailed { false };
    std::atomic<bool> isBusy { false };
    std::thread worker;
};
