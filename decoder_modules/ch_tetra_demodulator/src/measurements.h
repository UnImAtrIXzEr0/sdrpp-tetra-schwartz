#pragma once
// TETRA measurement collector: samples demodulator and decoder statistics once
// per second in its own thread (independent of whether the GUI menu is open),
// keeps the latest snapshot for the GUI and optionally appends it to a CSV file.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "dsp/pi4dqpsk.h"
#include "dsp/dqpsk_sym_extr.h"
#include "dsp/osmotetra_dec.h"
#include "subscribers.h"
#include "gps.h"
#include "trend.h"

#define TETRA_PLUGIN_VERSION "0.23.5"

struct TetraMeasSnapshot {
    bool valid = false;
    time_t timestamp = 0;

    // RF / demodulator
    float levelDbfs = -200.0f;
    float peakDbfs = -200.0f;
    double inputRate = 0.0;      // samples/s actually arriving at the demodulator (60 s regression)
    double inputRatePpm = 0.0;   // deviation from the nominal 72 kS/s
    double inputRateUncPpm = 0.0; // 1-sigma uncertainty of inputRatePpm
    double inputRateWindowS = 0.0;
    float merDb = 0.0f;
    float merExclDb = 0.0f;      // MER with I/Q origin offset removed
    float iqOffsetDb = -100.0f;  // I/Q origin offset (carrier leakage / RX DC) in dB rel. signal
    double vfoFreqHz = 0.0;      // tuned VFO frequency
    bool absErrValid = false;    // received carrier vs. SYSINFO DL frequency
    double absErrHz = 0.0;
    float evmPct = 100.0f;
    float evmPeakPct = 100.0f;
    float carrierOffsetHz = 0.0f;
    float carrierPpm = 0.0f;
    double refFreqHz = 0.0;
    bool refFromSysinfo = false;
    float symbolRatePpm = 0.0f;
    bool symSync = false;
    bool fllTracking = false;

    // Burst level
    int rxState = 0; // 0 unlocked, 1 know frame start, 2 locked
    uint32_t bursts = 0;           // in the last interval
    uint32_t tolerated = 0;        // in the last interval
    uint32_t syncLost = 0;         // in the last interval
    double ber = -1.0;             // last interval, -1 = no data
    double berTotal = -1.0;        // since reset
    uint32_t syncLostTotal = 0;
    uint32_t resyncSlip = 0, resyncNoSlip = 0;  // since reset
    int lastSlipBits = 0;

    // CRC per channel: 0=BSCH 1=SB2/BNCH 2=half-slot 3=SCH/F signalling
    uint32_t crcOk[4] = { 0, 0, 0, 0 };
    uint32_t crcFail[4] = { 0, 0, 0, 0 };
    uint32_t crcOkTotal[4] = { 0, 0, 0, 0 };
    uint32_t crcFailTotal[4] = { 0, 0, 0, 0 };

    // Network
    int mcc = 0, mnc = 0, cc = 0;
    int dlFreq = 0, ulFreq = 0;
    int ts[4] = { 0, 0, 0, 0 };
    // Timeslot occupancy from the AACH (frames 1..17), per timeslot:
    // DL 0 unallocated, 1 assigned control, 2 common control, 3 traffic;
    // UL 0 unallocated, 1 control, 2 traffic. Last second and since reset.
    uint32_t dlUse[4][4] = {};
    uint32_t ulUse[4][3] = {};
    uint32_t dlUseTotal[4][4] = {};
    uint32_t ulUseTotal[4][3] = {};

    // Position (GPS mouse)
    GpsFix gps;

    // Cell lock: -1 = no lock set, 0 = other/no cell received, 1 = locked cell
    int cellMatch = -1;
    float levelCalDb = 0.0f;
};

class TetraMeasurements {
public:
    ~TetraMeasurements() { stop(); }

    // vfoFreq returns the current VFO center frequency in Hz (reference when no SYSINFO)
    void start(dsp::demod::PI4DQPSK* demod, dsp::DQPSKSymbolExtractor* extr, dsp::osmotetradec* dec,
               std::function<double()> vfoFreq, std::function<bool()> decoderActive) {
        stop();
        _demod = demod;
        _extr = extr;
        _dec = dec;
        _vfoFreq = vfoFreq;
        _decoderActive = decoderActive;
        havePrev = false;
        syncSeconds = 0;
        running = true;
        worker = std::thread(&TetraMeasurements::run, this);
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lck(cvMtx);
            running = false;
        }
        cv.notify_all();
        if (worker.joinable()) { worker.join(); }
        stopLog();
    }

    TetraMeasSnapshot get() {
        std::lock_guard<std::mutex> lck(snapMtx);
        return snap;
    }

    void resetTotals() { resetRequested = true; }

    // long-term history (one point per second, 24 h)
    TetraTrendStore& trend() { return trendStore; }

    // ---- event log (sync loss / re-lock / gain and AFC steps) ----
    struct Event {
        time_t t;
        std::string text;
    };
    void addEvent(const std::string& text) {
        Event e { time(nullptr), text };
        {
            std::lock_guard<std::mutex> lck(evMtx);
            events.push_back(e);
            if (events.size() > 200) { events.erase(events.begin()); }
        }
        std::lock_guard<std::mutex> lck(logMtx);
        if (eventFile) {
            struct tm lt;
#ifdef _WIN32
            localtime_s(&lt, &e.t);
#else
            localtime_r(&e.t, &lt);
#endif
            char ts[32];
            strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &lt);
            fprintf(eventFile, "%s;%s\n", ts, text.c_str());
            fflush(eventFile);
        }
    }
    std::vector<Event> getEvents() {
        std::lock_guard<std::mutex> lck(evMtx);
        return events;
    }
    void clearEvents() {
        std::lock_guard<std::mutex> lck(evMtx);
        events.clear();
    }

    void setGps(NmeaGps* gps) { _gps = gps; }
    void setLevelCal(float db) { levelCal.store(db); }

    // Cell lock for coverage measurements (0 = don't care for that field)
    struct CellLock {
        bool active = false;
        int mcc = 0, mnc = 0, cc = 0;
        int dlFreq = 0;
    };
    void setCellLock(const CellLock& l) {
        std::lock_guard<std::mutex> lck(lockMtx);
        cellLock = l;
    }
    CellLock getCellLock() {
        std::lock_guard<std::mutex> lck(lockMtx);
        return cellLock;
    }

    // Station log: first appearance of every ISSI while logging (amateur mode)
    void setStationSource(TetraSubscriberTable* subs, std::function<std::string(uint32_t)> describe, std::function<bool()> enabled) {
        std::lock_guard<std::mutex> lck(logMtx);
        _subs = subs;
        _describe = describe;
        _stationsEnabled = enabled;
    }

    // --- CSV logging ---
    bool startLog(const std::string& dir, std::string& errOut) {
        std::lock_guard<std::mutex> lck(logMtx);
        if (logFile) { return true; }
        time_t now = time(nullptr);
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &now);
#else
        localtime_r(&now, &lt);
#endif
        char name[64];
        strftime(name, sizeof(name), "tetra_%Y%m%d_%H%M%S.csv", &lt);
        logPath = dir + "/" + name;
        logFile = fopen(logPath.c_str(), "w");
        if (!logFile) {
            errOut = "Cannot open " + logPath;
            return false;
        }
        fprintf(logFile, "# TETRA demodulator v" TETRA_PLUGIN_VERSION "\n");
        fprintf(logFile, "time;level_dbfs;mer_db;evm_rms_pct;evm_peak_pct;carrier_offset_hz;carrier_ppm;ref_freq_hz;"
                         "symbol_rate_ppm;rx_state;bursts;bursts_tolerated;sync_lost;ber;"
                         "crc_bsch_ok;crc_bsch_fail;crc_sb2_ok;crc_sb2_fail;crc_ndb_ok;crc_ndb_fail;crc_schf_ok;crc_schf_fail;"
                         "mcc;mnc;cc;dl_freq_hz;ul_freq_hz;ts1;ts2;ts3;ts4;"
                         "level_dbm;cell_match;gps_fix;lat;lon;alt_m;speed_kmh;sats;hdop;"
                         "ts1_traffic_pct;ts2_traffic_pct;ts3_traffic_pct;ts4_traffic_pct;carrier_vs_sysinfo_hz\n");
        fflush(logFile);
        eventFile = fopen((logPath.substr(0, logPath.size() - 4) + "_events.csv").c_str(), "w");
        if (eventFile) { fprintf(eventFile, "time;event\n"); fflush(eventFile); }
        if (_subs) {
            _subs->takeNew(); // only log stations appearing from now on
            stationPath = logPath.substr(0, logPath.size() - 4) + "_stations.csv";
            stationFile = fopen(stationPath.c_str(), "w");
            _subs->takeNewTraffic(); // only log traffic from now on
            trafficFile = fopen((logPath.substr(0, logPath.size() - 4) + "_traffic.csv").c_str(), "w");
            if (trafficFile) { fprintf(trafficFile, "time;ts;pdu;from;to;to_is_group;content;lat;lon\n"); fflush(trafficFile); }
            if (stationFile) {
                fprintf(stationFile, "time;issi;callsign;info\n");
                fflush(stationFile);
            }
        }
        return true;
    }

    void stopLog() {
        std::lock_guard<std::mutex> lck(logMtx);
        if (logFile) {
            fclose(logFile);
            logFile = nullptr;
        }
        if (stationFile) {
            fclose(stationFile);
            stationFile = nullptr;
        }
        if (eventFile) {
            fclose(eventFile);
            eventFile = nullptr;
        }
        if (trafficFile) {
            fclose(trafficFile);
            trafficFile = nullptr;
        }
    }

    bool isLogging() {
        std::lock_guard<std::mutex> lck(logMtx);
        return logFile != nullptr;
    }

    std::string getLogPath() {
        std::lock_guard<std::mutex> lck(logMtx);
        return logPath;
    }

private:
    void run() {
        auto next = std::chrono::steady_clock::now();
        while (true) {
            next += std::chrono::seconds(1);
            {
                std::unique_lock<std::mutex> lck(cvMtx);
                if (cv.wait_until(lck, next, [this] { return !running; })) { break; }
            }
            sample();
        }
    }

    void sample() {
        TetraMeasSnapshot s;
        s.valid = true;
        s.timestamp = time(nullptr);

        s.levelDbfs = _demod->getLevelDbfs();
        s.peakDbfs = _demod->getPeakDbfs();
        {
            // Input sample rate over a sliding 10 s window: missing samples
            // (USB drops, CPU overload) show up as a negative deviation.
            auto tnow = std::chrono::steady_clock::now();
            uint64_t cnt = _demod->getSampleCount();
            // a stopped / restarted source (e.g. sample rate change) is not a
            // sample loss: start the regression over instead of mixing the gap in
            if (!rateHist.empty()) {
                double dt = std::chrono::duration<double>(tnow - rateHist.back().first).count();
                double inc = (double)(cnt - rateHist.back().second);
                if (cnt < rateHist.back().second || (dt > 0.2 && inc < 0.2 * 72000.0 * dt)) {
                    rateHist.clear();
                }
            }
            rateHist.push_back({ tnow, cnt });
            // 60 s window, least-squares slope: SDRs deliver samples in large
            // blocks (Pluto: 20 ms), so a plain two-point rate jitters by
            // one block (+-2000 ppm over 10 s) without any real loss
            while (rateHist.size() > 61) { rateHist.erase(rateHist.begin()); }
            if (rateHist.size() >= 5) {
                const size_t n = rateHist.size();
                double mt = 0, mc = 0;
                std::vector<double> t(n), c(n);
                for (size_t i = 0; i < n; i++) {
                    t[i] = std::chrono::duration<double>(rateHist[i].first - rateHist[0].first).count();
                    c[i] = (double)(rateHist[i].second - rateHist[0].second);
                    mt += t[i]; mc += c[i];
                }
                mt /= n; mc /= n;
                double sxx = 0, sxy = 0;
                for (size_t i = 0; i < n; i++) { sxx += (t[i] - mt) * (t[i] - mt); sxy += (t[i] - mt) * (c[i] - mc); }
                if (sxx > 1.0) {
                    double slope = sxy / sxx;
                    double res = 0;
                    for (size_t i = 0; i < n; i++) { double e = c[i] - (mc + slope * (t[i] - mt)); res += e * e; }
                    double sigma = (n > 2) ? std::sqrt(res / (n - 2) / sxx) : 0.0;
                    s.inputRate = slope;
                    s.inputRatePpm = (slope / 72000.0 - 1.0) * 1e6;
                    s.inputRateUncPpm = sigma / 72000.0 * 1e6;
                    s.inputRateWindowS = t[n - 1];
                }
            }
        }
        s.carrierOffsetHz = _demod->getCarrierOffsetHz();
        s.symbolRatePpm = _demod->getSymbolRatePpm();
        s.merDb = _extr->getMerDb();
        s.merExclDb = _extr->getMerExclOffsetDb();
        s.iqOffsetDb = _extr->getIqOffsetDb();
        s.evmPct = _extr->getEvmPct();
        s.evmPeakPct = _extr->getEvmPeakPct();
        s.symSync = _extr->sync;
        // Narrow the FLL after 2 s of stable symbol sync, widen it on loss
        syncSeconds = s.symSync ? syncSeconds + 1 : 0;
        _demod->setTracking(syncSeconds >= 2);
        s.fllTracking = _demod->isTracking();

        bool decActive = _decoderActive ? _decoderActive() : true;
        if (decActive) {
            s.rxState = _dec->getRxState();
            uint32_t bursts = _dec->getStatBursts();
            uint32_t kb = _dec->getStatKnownBits();
            uint32_t ke = _dec->getStatKnownErr();
            uint32_t tol = _dec->getStatTolerated();
            uint32_t lost = _dec->getStatSyncLost();
            uint32_t ok[4], fail[4];
            for (int i = 0; i < 4; i++) {
                ok[i] = _dec->getCrcOk(i);
                fail[i] = _dec->getCrcFail(i);
            }

            if (resetRequested.exchange(false) || !havePrev) {
                resetFlagForSlip = true;
                useReset = true;
                // Start (or restart) totals from the current counter values
                baseKb = kb; baseKe = ke; baseLost = lost;
                for (int i = 0; i < 4; i++) { baseOk[i] = ok[i]; baseFail[i] = fail[i]; }
            }
            if (havePrev) {
                // unsigned subtraction is wrap-safe
                s.bursts = bursts - prevBursts;
                s.tolerated = tol - prevTol;
                s.syncLost = lost - prevLost;
                uint32_t dkb = kb - prevKb, dke = ke - prevKe;
                s.ber = dkb ? (double)dke / (double)dkb : -1.0;
                for (int i = 0; i < 4; i++) {
                    s.crcOk[i] = ok[i] - prevOk[i];
                    s.crcFail[i] = fail[i] - prevFail[i];
                }
            }
            uint32_t tkb = kb - baseKb, tke = ke - baseKe;
            s.berTotal = tkb ? (double)tke / (double)tkb : -1.0;
            s.syncLostTotal = lost - baseLost;
            {
                uint32_t rs = _dec->getStatResyncSlip(), rn = _dec->getStatResyncNoSlip(), re = _dec->getStatResyncEvents();
                if (!evInit || resetFlagForSlip) { baseSlip = rs; baseNoSlip = rn; }
                s.resyncSlip = rs - baseSlip;
                s.resyncNoSlip = rn - baseNoSlip;
                s.lastSlipBits = _dec->getLastSlipBits();
                if (evInit) {
                    if (lost != prevLostEv) {
                        addEvent("SYNC LOST x" + std::to_string(lost - prevLostEv) +
                                 " (MER " + std::to_string((int)std::lround(s.merDb)) + " dB, level " +
                                 std::to_string((int)std::lround(s.levelDbfs)) + " dBFS)");
                    }
                    if (re != prevResync) {
                        int slip = _dec->getLastSlipBits();
                        addEvent(std::string("RE-LOCK after ") + std::to_string(_dec->getLastResyncBits() / 510) + " slots, " +
                                 (slip ? std::string("timing slip -> samples lost/inserted (USB/CPU/SDR glitch)")
                                       : "no timing slip -> burst corrupted (signal)"));
                    }
                }
                prevLostEv = lost;
                prevResync = re;
                evInit = true;
                resetFlagForSlip = false;
            }
            for (int i = 0; i < 4; i++) {
                s.crcOkTotal[i] = ok[i] - baseOk[i];
                s.crcFailTotal[i] = fail[i] - baseFail[i];
            }
            prevBursts = bursts; prevKb = kb; prevKe = ke; prevTol = tol; prevLost = lost;
            for (int i = 0; i < 4; i++) { prevOk[i] = ok[i]; prevFail[i] = fail[i]; }
            havePrev = true;

            s.mcc = _dec->getMcc();
            s.mnc = _dec->getMnc();
            s.cc = _dec->getCc();
            s.dlFreq = _dec->getDlFreq();
            s.ulFreq = _dec->getUlFreq();
            for (int i = 0; i < 4; i++) { s.ts[i] = _dec->getTimeslotContent(i); }
            for (int t = 0; t < 4; t++) {
                for (int c = 0; c < 4; c++) {
                    uint32_t v = _dec->getDlUseCnt(t, c);
                    if (useInit && !useReset) { s.dlUse[t][c] = v - prevDlUse[t][c]; }
                    if (!useInit || useReset) { baseDlUse[t][c] = v; }
                    s.dlUseTotal[t][c] = v - baseDlUse[t][c];
                    prevDlUse[t][c] = v;
                }
                for (int c = 0; c < 3; c++) {
                    uint32_t v = _dec->getUlUseCnt(t, c);
                    if (useInit && !useReset) { s.ulUse[t][c] = v - prevUlUse[t][c]; }
                    if (!useInit || useReset) { baseUlUse[t][c] = v; }
                    s.ulUseTotal[t][c] = v - baseUlUse[t][c];
                    prevUlUse[t][c] = v;
                }
            }
            useInit = true;
            useReset = false;
        }

        if (_vfoFreq) { s.vfoFreqHz = _vfoFreq(); }
        if (s.rxState == 2 && s.dlFreq > 0 && s.vfoFreqHz > 1e6) {
            // independent of AFC: where the carrier really is vs. where SYSINFO says
            s.absErrHz = s.vfoFreqHz + s.carrierOffsetHz - (double)s.dlFreq;
            s.absErrValid = true;
        }
        // ppm reference: DL frequency from SYSINFO when locked, otherwise the VFO frequency
        if (s.rxState == 2 && s.dlFreq > 0) {
            s.refFreqHz = (double)s.dlFreq;
            s.refFromSysinfo = true;
        } else if (_vfoFreq) {
            s.refFreqHz = _vfoFreq();
        }
        s.levelCalDb = levelCal.load();
        if (_gps) { s.gps = _gps->get(); }
        {
            CellLock l = getCellLock();
            if (l.active) {
                bool ok = (s.rxState == 2) && (s.mcc == l.mcc) && (s.mnc == l.mnc) && (s.cc == l.cc) &&
                          (l.dlFreq == 0 || std::abs(s.dlFreq - l.dlFreq) < 1000);
                s.cellMatch = ok ? 1 : 0;
            }
        }
        if (s.refFreqHz > 1e6) {
            s.carrierPpm = (float)((double)s.carrierOffsetHz / s.refFreqHz * 1e6);
        }

        {
            std::lock_guard<std::mutex> lck(snapMtx);
            snap = s;
        }
        addTrend(s);
        writeLog(s);
    }

    void writeLog(const TetraMeasSnapshot& s) {
        std::lock_guard<std::mutex> lck(logMtx);
        if (!logFile) { return; }
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &s.timestamp);
#else
        localtime_r(&s.timestamp, &lt);
#endif
        char ts[32];
        strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &lt);
        fprintf(logFile, "%s;%.1f;%.2f;%.2f;%.1f;%.1f;%.3f;%.0f;%.2f;%d;%u;%u;%u;%.3e;"
                         "%u;%u;%u;%u;%u;%u;%u;%u;%d;%d;%d;%d;%d;%d;%d;%d;%d;",
                ts, s.levelDbfs, s.merDb, s.evmPct, s.evmPeakPct, s.carrierOffsetHz, s.carrierPpm, s.refFreqHz,
                s.symbolRatePpm, s.rxState, s.bursts, s.tolerated, s.syncLost, s.ber,
                s.crcOk[0], s.crcFail[0], s.crcOk[1], s.crcFail[1], s.crcOk[2], s.crcFail[2], s.crcOk[3], s.crcFail[3],
                s.mcc, s.mnc, s.cc, s.dlFreq, s.ulFreq, s.ts[0], s.ts[1], s.ts[2], s.ts[3]);
        if (s.levelCalDb != 0.0f) { fprintf(logFile, "%.1f;", s.levelDbfs + s.levelCalDb); }
        else { fprintf(logFile, ";"); }
        fprintf(logFile, "%d;", s.cellMatch);
        if (s.gps.valid) {
            fprintf(logFile, "1;%.6f;%.6f;%.1f;%.1f;%d;%.1f;", s.gps.lat, s.gps.lon, s.gps.altM, s.gps.speedKmh, s.gps.sats, s.gps.hdop);
        }
        else {
            fprintf(logFile, "0;;;;;%d;;", s.gps.sats);
        }
        for (int t = 0; t < 4; t++) {
            float pct = trafficPct(s.dlUse[t]);
            if (std::isnan(pct)) { fprintf(logFile, ";"); }
            else { fprintf(logFile, "%.0f;", pct); }
        }
        if (s.absErrValid) { fprintf(logFile, "%.2f\n", s.absErrHz); } else { fprintf(logFile, "\n"); }
        if (trafficFile && _subs) {
            bool en = _stationsEnabled ? _stationsEnabled() : true;
            auto tr = _subs->takeNewTraffic();
            if (en) {
                for (auto& e : tr) {
                    struct tm tl;
#ifdef _WIN32
                    localtime_s(&tl, &e.t);
#else
                    localtime_r(&e.t, &tl);
#endif
                    char tts[32];
                    strftime(tts, sizeof(tts), "%Y-%m-%dT%H:%M:%S", &tl);
                    std::string txt = e.text;
                    for (auto& c : txt) { if (c == ';' || c == '\n') { c = ','; } }
                    if (e.hasPos) {
                        fprintf(trafficFile, "%s;%u;%s;%u;%u;%d;%s;%.6f;%.6f\n", tts, e.tn, e.pdu.c_str(), e.from, e.to, e.toGroup ? 1 : 0, txt.c_str(), e.lat, e.lon);
                    } else {
                        fprintf(trafficFile, "%s;%u;%s;%u;%u;%d;%s;;\n", tts, e.tn, e.pdu.c_str(), e.from, e.to, e.toGroup ? 1 : 0, txt.c_str());
                    }
                }
                if (!tr.empty()) { fflush(trafficFile); }
            }
        }
        if (stationFile && _subs) {
            bool en = _stationsEnabled ? _stationsEnabled() : true;
            std::vector<uint32_t> fresh = _subs->takeNew();
            if (en) {
                for (uint32_t ssi : fresh) {
                    std::string d = _describe ? _describe(ssi) : std::string(";");
                    fprintf(stationFile, "%s;%u;%s\n", ts, ssi, d.c_str());
                }
                if (!fresh.empty()) { fflush(stationFile); }
            }
        }
        // flush every 10 lines to limit disk wakeups while staying crash-safe
        if (++linesSinceFlush >= 10) {
            fflush(logFile);
            linesSinceFlush = 0;
        }
    }

public:
    // share of the counted frames of one timeslot that carried traffic (%), NaN without data
    static float trafficPct(const uint32_t cnt[4]) {
        uint32_t n = cnt[0] + cnt[1] + cnt[2] + cnt[3];
        return n ? 100.0f * (float)cnt[3] / (float)n : NAN;
    }

private:
    void addTrend(const TetraMeasSnapshot& s) {
        TetraTrendPoint p;
        p.t = s.timestamp;
        p.calibrated = s.levelCalDb != 0.0f;
        p.level = s.levelDbfs + s.levelCalDb;
        if (s.rxState == 2 && s.symSync) {
            p.mer = s.merDb;
            p.evm = s.evmPct;
            p.carrierHz = s.carrierOffsetHz;
            if (s.absErrValid && s.dlFreq > 0) { p.ppb = (float)(s.absErrHz / (double)s.dlFreq * 1e9); }
        }
        bool decActive = _decoderActive ? _decoderActive() : true;
        if (decActive) {
            p.bursts = (float)s.bursts;
            p.syncLost = (float)s.syncLost;
            double ok = 0, fail = 0;
            for (int i = 0; i < 4; i++) { ok += s.crcOk[i]; fail += s.crcFail[i]; }
            if (ok + fail > 0) { p.crcFail = (float)(100.0 * fail / (ok + fail)); }
            for (int t = 0; t < 4; t++) { p.tsTraffic[t] = trafficPct(s.dlUse[t]); }
        }
        trendStore.add(p);
    }

    TetraTrendStore trendStore;
    uint32_t prevDlUse[4][4] = {}, baseDlUse[4][4] = {};
    uint32_t prevUlUse[4][3] = {}, baseUlUse[4][3] = {};
    bool useInit = false, useReset = false;

    dsp::demod::PI4DQPSK* _demod = nullptr;
    dsp::DQPSKSymbolExtractor* _extr = nullptr;
    dsp::osmotetradec* _dec = nullptr;
    std::function<double()> _vfoFreq;
    std::function<bool()> _decoderActive;
    NmeaGps* _gps = nullptr;
    std::atomic<float> levelCal { 0.0f };
    std::mutex lockMtx;
    CellLock cellLock;

    std::thread worker;
    std::mutex cvMtx;
    std::condition_variable cv;
    bool running = false;

    std::mutex snapMtx;
    TetraMeasSnapshot snap;

    std::atomic<bool> resetRequested { false };
    bool havePrev = false;
    bool resetFlagForSlip = false;
    std::vector<std::pair<std::chrono::steady_clock::time_point, uint64_t>> rateHist;
    int syncSeconds = 0;
    uint32_t prevBursts = 0, prevKb = 0, prevKe = 0, prevTol = 0, prevLost = 0;
    uint32_t prevOk[4] = { 0 }, prevFail[4] = { 0 };
    uint32_t baseKb = 0, baseKe = 0, baseLost = 0;
    uint32_t baseOk[4] = { 0 }, baseFail[4] = { 0 };

    std::mutex logMtx;
    FILE* logFile = nullptr;
    FILE* stationFile = nullptr;
    FILE* eventFile = nullptr;
    FILE* trafficFile = nullptr;
    std::mutex evMtx;
    std::vector<Event> events;
    uint32_t prevResync = 0, prevResyncSlip = 0, prevLostEv = 0, baseSlip = 0, baseNoSlip = 0;
    bool evInit = false;
    std::string stationPath;
    TetraSubscriberTable* _subs = nullptr;
    std::function<std::string(uint32_t)> _describe;
    std::function<bool()> _stationsEnabled;
    std::string logPath;
    int linesSinceFlush = 0;
};
