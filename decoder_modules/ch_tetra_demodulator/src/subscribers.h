#pragma once
// Table of addresses (ISSI/GSSI) seen in MAC-RESOURCE PDUs.
// Filled from the decoder thread, read by the GUI and the CSV logger.

#include <algorithm>
#include <cstdint>
#include <ctime>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <deque>

extern "C" {
#include "tetra_l3.h"
}

struct TetraSubscriber {
    uint32_t ssi = 0;
    time_t firstSeen = 0;
    time_t lastSeen = 0;
    uint32_t count = 0;
    uint8_t lastTn = 0;        // timeslot 1..4
    uint8_t lastUsageMarker = 0;
    uint8_t lastAddrType = 0;
    bool encrypted = false;    // MAC-RESOURCE encryption mode != 0
    bool chanAlloc = false;    // seen with a channel allocation (call setup / traffic)

    // From layer 3 decoding
    enum Kind { KIND_UNKNOWN = 0, KIND_INDIVIDUAL = 1, KIND_GROUP = 2, KIND_BROADCAST = 3 };
    int kind = KIND_UNKNOWN;
    std::string lastActivity;  // e.g. "talking in 26200", "SDS to ...", "registered"
    time_t lastActivityTime = 0;
    uint32_t talker = 0;       // groups: current talker ISSI (0 = nobody)
    uint32_t lastTalker = 0;   // groups: last talker
    uint32_t talkingIn = 0;    // individuals: group/ISSI currently talking in
    bool hasPos = false;
    double lat = 0.0, lon = 0.0;
    time_t posTime = 0;
    std::vector<uint32_t> groups;  // attached talkgroups (scan list), from MM group attach/detach
};

// One line of the traffic list
struct TetraTrafficEntry {
    time_t t = 0;
    uint8_t tn = 0;
    std::string pdu;
    uint32_t from = 0;     // calling / transmitting party (0 = unknown)
    uint32_t to = 0;       // MAC address
    bool toGroup = false;
    std::string text;
    bool hasPos = false;
    double lat = 0.0, lon = 0.0;
};

class TetraSubscriberTable {
public:
    static const size_t MAX_ENTRIES = 2048;

    // C callback trampoline, ctx = TetraSubscriberTable*
    static void onAddress(void* ctx, uint8_t addrType, uint32_t ssi, uint8_t usageMarker,
                          uint16_t eventLabel, uint8_t tn, uint8_t encrMode, uint8_t chanAlloc) {
        ((TetraSubscriberTable*)ctx)->add(addrType, ssi, usageMarker, tn, encrMode, chanAlloc);
    }

    void add(uint8_t addrType, uint32_t ssi, uint8_t usageMarker, uint8_t tn, uint8_t encrMode, uint8_t chanAlloc) {
        // Only address types that carry an SSI (SSI, USSI, SMI and the combined forms)
        bool hasSsi = (addrType == 1 || addrType == 3 || addrType == 4 || addrType == 5 || addrType == 6 || addrType == 7);
        if (!hasSsi || ssi == 0 || ssi > 0xFFFFFF) { return; }
        time_t now = time(nullptr);
        std::lock_guard<std::mutex> lck(mtx);
        TetraSubscriber& s = entry(ssi, now);
        s.lastSeen = now;
        s.count++;
        s.lastTn = tn;
        s.lastAddrType = addrType;
        if (addrType == 6) { s.lastUsageMarker = usageMarker; }
        s.encrypted = s.encrypted || (encrMode != 0);
        s.chanAlloc = s.chanAlloc || (chanAlloc != 0);
    }

    // ---- layer 3 events (decoder thread) ----
    static void onL3(void* ctx, const tetra_l3_event* ev) {
        ((TetraSubscriberTable*)ctx)->addL3(*ev);
    }

    void addL3(const tetra_l3_event& ev) {
        time_t now = time(nullptr);
        std::lock_guard<std::mutex> lck(mtx);
        TetraTrafficEntry te;
        te.t = now;
        te.tn = ev.tn;
        te.pdu = ev.pdu;
        te.from = ev.party_ssi;
        te.to = ev.addr_ssi;
        te.text = ev.text;
        te.hasPos = ev.has_pos != 0;
        te.lat = ev.lat;
        te.lon = ev.lon;

        bool addrValid = ev.addr_ssi != 0 && ev.addr_ssi <= 0xFFFFFF;
        // Classification (EN 300 392-2): the basic service's communication type
        // tells whether the call address is a group; calling/transmitting
        // parties and MM-addressed SSIs are always individual.
        if (addrValid && ev.comm_type >= 0 && (ev.type == TETRA_L3_CALL_SETUP || ev.type == TETRA_L3_CALL_CONNECT)) {
            setKind(ev.addr_ssi, ev.comm_type == 0 ? TetraSubscriber::KIND_INDIVIDUAL : TetraSubscriber::KIND_GROUP, now);
        }
        if (addrValid && ev.type == TETRA_L3_MM) {
            setKind(ev.addr_ssi, TetraSubscriber::KIND_INDIVIDUAL, now);
            TetraSubscriber& s = entry(ev.addr_ssi, now);
            s.lastActivity = std::string(ev.pdu) + (ev.text[0] ? std::string(": ") + ev.text : std::string());
            s.lastActivityTime = now;
            if (ev.ngroups > 0 || ev.detach_all) {
                if (ev.detach_all) { s.groups.clear(); }
                for (int i = 0; i < ev.ngroups && i < 16; i++) {
                    uint32_t g = ev.groups[i];
                    auto it = std::find(s.groups.begin(), s.groups.end(), g);
                    if (ev.group_detach[i]) {
                        if (it != s.groups.end()) { s.groups.erase(it); }
                    } else if (it == s.groups.end()) {
                        s.groups.push_back(g);
                    }
                }
                std::vector<uint32_t> gs = s.groups;   // s may move when entries are added
                uint32_t member = ev.addr_ssi;
                for (int i = 0; i < ev.ngroups && i < 16; i++) { setKind(ev.groups[i], TetraSubscriber::KIND_GROUP, now); }
                (void)gs; (void)member;
            }
        }
        if (ev.party_ssi) { setKind(ev.party_ssi, TetraSubscriber::KIND_INDIVIDUAL, now); }
        if (ev.call_id >= 0 && addrValid && (ev.type == TETRA_L3_CALL_SETUP || ev.type == TETRA_L3_CALL_CONNECT || ev.type == TETRA_L3_TX_GRANTED)) {
            calls[ev.call_id] = ev.addr_ssi;
        }
        auto callAddr = [&](int cid) -> uint32_t {
            auto it = calls.find(cid);
            return it == calls.end() ? 0 : it->second;
        };

        switch (ev.type) {
        case TETRA_L3_CALL_SETUP:
        case TETRA_L3_TX_GRANTED:
            if (ev.party_ssi && addrValid) {
                TetraSubscriber& g = entry(ev.addr_ssi, now);
                if (g.talker && g.talker != ev.party_ssi) {
                    auto pit = table.find(g.talker);
                    if (pit != table.end() && pit->second.talkingIn == ev.addr_ssi) { pit->second.talkingIn = 0; }
                }
                g.talker = ev.party_ssi;
                g.lastTalker = ev.party_ssi;
                g.lastActivity = "talker " + std::to_string(ev.party_ssi);
                g.lastActivityTime = now;
                TetraSubscriber& p = entry(ev.party_ssi, now);
                p.talkingIn = ev.addr_ssi;
                p.lastActivity = std::string(ev.type == TETRA_L3_CALL_SETUP ? "call setup to " : "talking in ") + std::to_string(ev.addr_ssi);
                p.lastActivityTime = now;
                talkerOfCall[ev.call_id] = ev.party_ssi;
            }
            if (ev.type == TETRA_L3_CALL_SETUP) {
                te.text = std::string(ev.comm_type == 0 ? "individual call" : ev.comm_type > 0 ? "group call" : "call") +
                          (ev.encrypted_call == 1 ? " (encrypted)" : "") + (te.text.empty() ? "" : " " + te.text);
            }
            break;
        case TETRA_L3_TX_CEASED:
        case TETRA_L3_CALL_RELEASE: {
            uint32_t a = addrValid ? ev.addr_ssi : callAddr(ev.call_id);
            if (!a) { a = callAddr(ev.call_id); }
            if (a) {
                TetraSubscriber& g = entry(a, now);
                g.talker = 0;
                for (auto& kv : table) {
                    if (kv.second.talkingIn == a) { kv.second.talkingIn = 0; }
                }
                te.to = a;
            }
            auto tit = talkerOfCall.find(ev.call_id);
            if (tit != talkerOfCall.end()) {
                if (!te.from) { te.from = tit->second; }
                if (ev.type == TETRA_L3_CALL_RELEASE) { talkerOfCall.erase(tit); }
            }
            if (ev.type == TETRA_L3_CALL_RELEASE) {
                calls.erase(ev.call_id);
                if (ev.disconnect_cause >= 0) { te.text = "cause " + std::to_string(ev.disconnect_cause); }
            }
            break;
        }
        case TETRA_L3_SDS:
        case TETRA_L3_STATUS:
            if (ev.party_ssi) {
                TetraSubscriber& p = entry(ev.party_ssi, now);
                p.lastActivity = std::string(ev.type == TETRA_L3_SDS ? "SDS" : "status") + " to " + std::to_string(ev.addr_ssi) +
                                 (ev.text[0] ? std::string(": ") + ev.text : std::string());
                p.lastActivityTime = now;
                if (ev.has_pos) {
                    p.hasPos = true;
                    p.lat = ev.lat;
                    p.lon = ev.lon;
                    p.posTime = now;
                }
            }
            break;
        default:
            break;
        }
        if (addrValid) {
            auto it = table.find(ev.addr_ssi);
            if (it != table.end()) { te.toGroup = (it->second.kind == TetraSubscriber::KIND_GROUP); }
        }
        traffic.push_back(te);
        while (traffic.size() > MAX_TRAFFIC) { traffic.pop_front(); }
        if (newTraffic.size() < 4096) { newTraffic.push_back(te); }
    }

    std::vector<TetraTrafficEntry> trafficSnapshot() {
        std::lock_guard<std::mutex> lck(mtx);
        return std::vector<TetraTrafficEntry>(traffic.begin(), traffic.end());
    }

    std::vector<TetraTrafficEntry> takeNewTraffic() {
        std::lock_guard<std::mutex> lck(mtx);
        std::vector<TetraTrafficEntry> v;
        v.swap(newTraffic);
        return v;
    }

    void clearTraffic() {
        std::lock_guard<std::mutex> lck(mtx);
        traffic.clear();
    }

    // Newest activity first
    std::vector<TetraSubscriber> snapshot() {
        std::vector<TetraSubscriber> v;
        {
            std::lock_guard<std::mutex> lck(mtx);
            v.reserve(table.size());
            for (auto& kv : table) { v.push_back(kv.second); }
        }
        std::sort(v.begin(), v.end(), [](const TetraSubscriber& a, const TetraSubscriber& b) {
            return a.lastSeen != b.lastSeen ? a.lastSeen > b.lastSeen : a.ssi < b.ssi;
        });
        return v;
    }

    // SSIs that appeared for the first time since the last call (for the log)
    std::vector<uint32_t> takeNew() {
        std::lock_guard<std::mutex> lck(mtx);
        std::vector<uint32_t> v;
        v.swap(newSinceLog);
        return v;
    }

    void clear() {
        std::lock_guard<std::mutex> lck(mtx);
        table.clear();
        newSinceLog.clear();
        calls.clear();
        talkerOfCall.clear();
    }

    size_t size() {
        std::lock_guard<std::mutex> lck(mtx);
        return table.size();
    }

private:
    static const size_t MAX_TRAFFIC = 1000;

    TetraSubscriber& entry(uint32_t ssi, time_t now) {
        auto it = table.find(ssi);
        if (it == table.end()) {
            if (table.size() >= MAX_ENTRIES) { evictOldest(); }
            TetraSubscriber s;
            s.ssi = ssi;
            s.firstSeen = now;
            s.lastSeen = now;
            if (ssi == 0xFFFFFF) { s.kind = TetraSubscriber::KIND_BROADCAST; }
            it = table.emplace(ssi, s).first;
            if (newSinceLog.size() < 4096) { newSinceLog.push_back(ssi); }
        }
        return it->second;
    }

    void setKind(uint32_t ssi, int kind, time_t now) {
        if (ssi == 0 || ssi == 0xFFFFFF) { return; }
        TetraSubscriber& s = entry(ssi, now);
        s.kind = kind;
        s.lastSeen = now;
    }

    void evictOldest() {
        auto oldest = table.begin();
        for (auto it = table.begin(); it != table.end(); ++it) {
            if (it->second.lastSeen < oldest->second.lastSeen) { oldest = it; }
        }
        if (oldest != table.end()) { table.erase(oldest); }
    }

    std::mutex mtx;
    std::unordered_map<uint32_t, TetraSubscriber> table;
    std::vector<uint32_t> newSinceLog;
    std::deque<TetraTrafficEntry> traffic;
    std::vector<TetraTrafficEntry> newTraffic;
    std::unordered_map<int, uint32_t> calls;        // call id -> call address
    std::unordered_map<int, uint32_t> talkerOfCall; // call id -> current talker
};
