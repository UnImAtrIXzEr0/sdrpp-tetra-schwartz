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
    uint8_t allocTs = 0;       // timeslot 1..4 of the last downlink channel allocation (clear signalling)
    time_t allocTime = 0;
    bool e2ee = false;         // call with end-to-end encryption (basic service information)

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
        time_t now = nowT();
        std::lock_guard<std::mutex> lck(mtx);
        TetraSubscriber& s = entry(ssi, now);
        // channel allocation of this MAC-RESOURCE (0 = none / not readable): the
        // layer 3 PDU of the same MAC-RESOURCE follows right after (addL3)
        lastPduSsi = ssi;
        lastPduAllocTs = 0;
        s.lastSeen = now;
        s.count++;
        s.lastTn = tn;
        s.lastAddrType = addrType;
        if (addrType == 6) { s.lastUsageMarker = usageMarker; }
        s.encrypted = s.encrypted || (encrMode != 0);
        s.chanAlloc = s.chanAlloc || (chanAlloc != 0);
        if (chanAlloc & 0x0f) {   // timeslot bitmap, TS1 = 8
            for (int t = 0; t < 4; t++) { if (chanAlloc & (8 >> t)) { s.allocTs = (uint8_t)(t + 1); break; } }
            s.allocTime = now;
            lastPduAllocTs = s.allocTs;
        }
        if (encrMode != 0) {
            // air interface encryption on the address of a running call: its timeslot is encrypted
            for (int t = 0; t < 4; t++) {
                auto c = callRec.find(slotCall[t]);
                if (slotCall[t] >= 0 && c != callRec.end() && c->second.addr == ssi) { c->second.enc = 1; }
            }
            encryptedPdus++;
            lastEncrTime = now;
            // one traffic line per address and 10 s: the content is not decoded
            auto it = lastEncrLine.find(ssi);
            if (it == lastEncrLine.end() || now - it->second >= 10) {
                lastEncrLine[ssi] = now;
                TetraTrafficEntry te;
                te.t = now;
                te.tn = tn;
                te.pdu = "encrypted";
                te.to = ssi;
                te.text = "Encrypted (air interface encryption mode " + std::to_string(encrMode) + ") - not decoded, this program does not decrypt";
                traffic.push_back(te);
                while (traffic.size() > MAX_TRAFFIC) { traffic.pop_front(); }
            }
        }
    }
    // "Listen to": which address owns timeslot ts (1..4) - the newest downlink
    // channel allocation within 'maxAge' seconds; 0 if none
    struct SlotOwner { uint32_t ssi = 0; bool e2ee = false; bool air = false; uint32_t talker = 0; };
    SlotOwner slotOwner(int ts, long maxAge = 60) {
        std::lock_guard<std::mutex> lck(mtx);
        SlotOwner o;
        time_t now = nowT(), best = 0;
        for (auto& kv : table) {
            const TetraSubscriber& x = kv.second;
            if (x.allocTs == ts && now - x.allocTime <= maxAge && x.allocTime > best) {
                best = x.allocTime;
                o.ssi = x.ssi; o.e2ee = x.e2ee; o.air = x.encrypted; o.talker = x.talker;
            }
        }
        return o;
    }
    // "Mute encrypted": encryption state of the call on timeslot ts (1..4).
    // Kept per timeslot for the current call, linked by the call identifier:
    // set by D-SETUP / D-CONNECT (basic service information, EN 300 392-2
    // 14.8.2), confirmed by D-TX-GRANTED of the same call, cleared by
    // D-RELEASE / D-DISCONNECT - independent of the age of the channel
    // allocation, so a long call with several transmissions keeps its state.
    // A timeslot without a known call (call setup missed: started before
    // decoding, CRC error) is LISTEN_UNKNOWN; the caller mutes it.
    // Idle timeout: a call state not confirmed by any PDU of that call for
    // CALL_IDLE_S falls back to unknown, so a stale "clear" (its D-RELEASE and
    // the next D-SETUP on the timeslot both missed) cannot keep a slot audible.
    enum ListenEnc { LISTEN_CLEAR = 0, LISTEN_ENCRYPTED = 1, LISTEN_UNKNOWN = 2 };
    // EN 300 392-2 V3.8.1 14.6, table 14.3 (protocol timers): one transmission may last up to
    // T311 = 300 s (call transmission timer, maximum) without any further PDU of
    // the call; every new transmission starts with D-TX-GRANTED and the hang
    // time ends with D-TX-GRANTED or D-RELEASE. 300 s + 20 s margin.
    static constexpr long CALL_IDLE_S = 320;
    int listenEncState(int ts) {
        SlotOwner o = slotOwner(ts);
        std::lock_guard<std::mutex> lck(mtx);
        int enc = -1;
        if (ts >= 1 && ts <= 4 && slotCall[ts - 1] >= 0) {
            auto c = callRec.find(slotCall[ts - 1]);
            if (c != callRec.end() && nowT() - c->second.seen > CALL_IDLE_S) {
                slotCall[ts - 1] = -1;   // stale: the call is no longer bound to the timeslot
            }
            else if (c != callRec.end()) { enc = c->second.enc; }
        }
        // everything the allocation-based owner marks as encrypted stays muted
        if (enc == 1 || (o.ssi && (o.e2ee || o.air))) { return LISTEN_ENCRYPTED; }
        return enc == 0 ? LISTEN_CLEAR : LISTEN_UNKNOWN;
    }
    // call identifier on timeslot ts (1..4), -1 = none known
    int slotCallId(int ts) {
        std::lock_guard<std::mutex> lck(mtx);
        return (ts >= 1 && ts <= 4) ? slotCall[ts - 1] : -1;
    }
    // timeslot of an address (group, or the group a station talks in / its own call); 0 = none
    int slotOf(uint32_t ssi, bool station, long maxAge = 60) {
        std::lock_guard<std::mutex> lck(mtx);
        time_t now = nowT();
        auto ok = [&](const TetraSubscriber& x) { return x.allocTs && now - x.allocTime <= maxAge; };
        auto it = table.find(ssi);
        if (!station) { return (it != table.end() && ok(it->second)) ? it->second.allocTs : 0; }
        if (it != table.end() && it->second.talkingIn) {
            auto g = table.find(it->second.talkingIn);
            if (g != table.end() && ok(g->second)) { return g->second.allocTs; }
        }
        return (it != table.end() && ok(it->second)) ? it->second.allocTs : 0;
    }
    time_t lastEncrypted() {
        std::lock_guard<std::mutex> lck(mtx);
        return lastEncrTime;
    }

    uint64_t encryptedCount() {
        std::lock_guard<std::mutex> lck(mtx);
        return encryptedPdus;
    }

    // ---- layer 3 events (decoder thread) ----
    static void onL3(void* ctx, const tetra_l3_event* ev) {
        ((TetraSubscriberTable*)ctx)->addL3(*ev);
    }

    void addL3(const tetra_l3_event& ev) {
        time_t now = nowT();
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
        trackCallSlot(ev, addrValid, now);
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
            if (ev.type == TETRA_L3_CALL_SETUP && addrValid && ev.encrypted_call >= 0) {
                entry(ev.addr_ssi, now).e2ee = ev.encrypted_call == 1;
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
        callRec.clear();
        for (int t = 0; t < 4; t++) { slotCall[t] = -1; }
        lastPduSsi = 0;
        lastPduAllocTs = 0;
    }

    // tests: fixed clock instead of time() (0 = real time)
    void setTestClock(time_t t) { testNow = t; }

    size_t size() {
        std::lock_guard<std::mutex> lck(mtx);
        return table.size();
    }

private:
    static const size_t MAX_TRAFFIC = 1000;
    static const size_t MAX_CALLS = 64;

    time_t nowT() const { return testNow ? testNow : time(nullptr); }

    // Per-timeslot call state for "Mute encrypted" (see listenEncState).
    // The traffic timeslot comes from the channel allocation of the
    // MAC-RESOURCE that carried the PDU (add() runs right before addL3() for
    // it); without one, a known call keeps its timeslot.
    void trackCallSlot(const tetra_l3_event& ev, bool addrValid, time_t now) {
        int allocTs = 0;
        if (addrValid && ev.addr_ssi == lastPduSsi) {
            allocTs = lastPduAllocTs;
            lastPduSsi = 0;
            lastPduAllocTs = 0;
        }
        if (ev.call_id < 0) { return; }
        if (ev.type == TETRA_L3_CALL_RELEASE) {
            for (int t = 0; t < 4; t++) { if (slotCall[t] == ev.call_id) { slotCall[t] = -1; } }
            callRec.erase(ev.call_id);
            return;
        }
        auto it = callRec.find(ev.call_id);
        if (ev.type == TETRA_L3_TX_CEASED || ev.type == TETRA_L3_CALL_OTHER) {
            // any other PDU of a known call (D-TX-CEASED, D-INFO, D-TX-CONTINUE ...) confirms it
            if (it != callRec.end()) { it->second.seen = now; }
            return;
        }
        if (ev.type != TETRA_L3_CALL_SETUP && ev.type != TETRA_L3_CALL_CONNECT && ev.type != TETRA_L3_TX_GRANTED) { return; }
        if (it == callRec.end()) {
            if (callRec.size() >= MAX_CALLS) {
                auto oldest = callRec.begin();
                for (auto c = callRec.begin(); c != callRec.end(); ++c) { if (c->second.seen < oldest->second.seen) { oldest = c; } }
                for (int t = 0; t < 4; t++) { if (slotCall[t] == oldest->first) { slotCall[t] = -1; } }
                callRec.erase(oldest);
            }
            it = callRec.emplace(ev.call_id, CallRec()).first;
        }
        CallRec& c = it->second;
        c.seen = now;
        if (addrValid) { c.addr = ev.addr_ssi; }
        // D-TX-GRANTED carries no basic service information: it keeps the state
        if (ev.type != TETRA_L3_TX_GRANTED && ev.encrypted_call >= 0) { c.enc = ev.encrypted_call ? 1 : 0; }
        if (allocTs) { c.ts = allocTs; }
        if (!c.ts) { return; }
        int& sc = slotCall[c.ts - 1];
        // an explicit channel allocation always takes the timeslot; without one the
        // call only takes it back when no other call holds it
        if (allocTs || sc < 0 || sc == ev.call_id) {
            for (int t = 0; t < 4; t++) { if (slotCall[t] == ev.call_id) { slotCall[t] = -1; } }
            sc = ev.call_id;
        }
    }

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
    std::unordered_map<uint32_t, time_t> lastEncrLine;
    uint64_t encryptedPdus = 0;
    time_t lastEncrTime = 0;
    std::vector<TetraTrafficEntry> newTraffic;
    std::unordered_map<int, uint32_t> calls;        // call id -> call address
    std::unordered_map<int, uint32_t> talkerOfCall; // call id -> current talker
    struct CallRec { int ts = 0; int enc = -1; uint32_t addr = 0; time_t seen = 0; };   // enc: -1 unknown, 0 clear, 1 encrypted
    std::unordered_map<int, CallRec> callRec;       // running calls (call id -> state)
    int slotCall[4] = { -1, -1, -1, -1 };           // call id on TS1..TS4, -1 = none known
    uint32_t lastPduSsi = 0;                        // address of the last MAC-RESOURCE
    uint8_t lastPduAllocTs = 0;                     // its downlink channel allocation timeslot (0 = none)
    time_t testNow = 0;                             // tests: fixed clock (setTestClock)
};
