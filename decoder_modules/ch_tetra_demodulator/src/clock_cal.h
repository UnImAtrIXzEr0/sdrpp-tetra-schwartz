#pragma once
// Calibration of the SDR's internal oscillator against its external reference
// (SourceClockCalInterface, requested in the source menu, e.g. USRP). The
// received TETRA carrier is the measuring object; its own accuracy does not
// matter, only that it stays the same:
//   1. on the reference: mean of (carrier - SYSINFO DL frequency) over MEAS_S
//   2. switch to the internal oscillator (current correction applied), wait
//      for the PLL to relock and the decoder to lock again, then measure again
//   3. difference / DL frequency = remaining error of the internal oscillator;
//      new correction = old correction - difference (as "Calibrate SDR")
//   4. store it in the source and switch back to the reference
// Any loss of lock while measuring aborts; the source is always switched back.
// Runs in the GUI thread (called every frame). step() has no SDR++
// dependencies (unit test: e2e/gen/test_clock_cal.cpp, which defines
// TETRA_CLOCK_CAL_STANDALONE).

#include <cmath>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>
#include <signal_path/source_clock_cal_interface.h>
#ifndef TETRA_CLOCK_CAL_STANDALONE
#include <module.h>
#include <signal_path/signal_path.h>
#include "measurements.h"
#endif

class TetraClockCal {
public:
    bool busy() const { return state != IDLE; }
    static constexpr double SETTLE_S = 4.0;     // after lock: PLL and carrier loop settled
    static constexpr double MEAS_S = 6.0;       // averaging time per clock state
    static constexpr double RELOCK_MAX_S = 20.0; // PLL relock + decoder lock after a clock switch
    static constexpr double MAX_STD_HZ = 5.0;   // spread of the carrier samples: else unstable
    static constexpr double MAX_PPM = 20.0;     // plausibility of the result

#ifndef TETRA_CLOCK_CAL_STANDALONE
    static SourceClockCalInterface* find() {
        std::string sel = sigpath::sourceManager.getSelectedName();
        auto list = core::moduleManager.getAllInterfaces<SourceClockCalInterface>("SourceClockCalInterface");
        for (auto* c : list) {
            if (c && c->clockCalSourceName() == sel) { return c; }
        }
        return nullptr;
    }

    // every GUI frame; 'active': TETRA Analyzer running in TMO decoding
    void loop(bool active, const TetraMeasSnapshot& m) {
        step(find(), active, m.absErrValid && m.refFreqHz > 1e6, m.absErrHz, m.refFreqHz, now());
    }
#endif

    // locked: decoder locked with a carrier error (carrier - SYSINFO DL frequency)
    // of errHz at the DL frequency refHz; t: monotonic time in seconds
    void step(SourceClockCalInterface* sc, bool active, bool locked, double errHz, double refHzNow, double t) {
        if (sc != src) {
            if (src && state != IDLE) { abortRun("source changed"); }
            src = sc;
        }
        if (!src) { return; }
        if (state == IDLE) {
            if (!src->clockCalTakeRequest()) { return; }
            src->clockCalTakeCancel();
            if (!active) { src->clockCalStatus("failed: the TETRA Analyzer is not running", false); return; }
            if (!src->clockCalPossible()) { src->clockCalStatus("failed: select the external reference / GPSDO first", false); return; }
            if (!locked) { src->clockCalStatus("failed: tune the TETRA Analyzer to a carrier and wait for lock", false); return; }
            refHz = refHzNow;
            enter(SETTLE_REF, t);
            src->clockCalStatus("measuring the carrier with the reference...", true);
            return;
        }
        if (src->clockCalTakeCancel()) { abortRun("cancelled"); return; }
        if (!active) { abortRun("the TETRA Analyzer was stopped"); return; }
        double dt = t - t0;
        switch (state) {
        case SETTLE_REF:
            if (!locked) { abortRun("lock lost on the reference"); return; }
            if (dt >= 1.0) { enter(MEAS_REF, t); }
            break;
        case MEAS_REF:
        case MEAS_INT:
            if (!locked) { abortRun(state == MEAS_REF ? "lock lost on the reference" : "lock lost on the internal clock"); return; }
            sample(errHz, t);
            if (dt >= MEAS_S) {
                double mean, sd;
                if (!stats(mean, sd)) { abortRun("too few measurements"); return; }
                if (sd > MAX_STD_HZ) { abortRun("carrier frequency unstable"); return; }
                if (state == MEAS_REF) {
                    errRefHz = mean;
                    oldPpm = src->clockCalPpm();
                    if (!src->clockCalUseInternal(true)) { abortRun("switching to the internal clock failed"); return; }
                    switched = true;
                    src->clockCalStatus("internal clock: waiting for the PLL and the decoder...", true);
                    enter(RELOCK, t);
                }
                else { finish(mean); }
            }
            break;
        case RELOCK:
            // the frequency jumps when the clock is switched: wait for a stable lock
            if (!locked) { lockT = -1.0; }
            else if (lockT < 0.0) { lockT = t; }
            if (lockT >= 0.0 && t - lockT >= SETTLE_S) {
                src->clockCalStatus("measuring the carrier with the internal clock...", true);
                enter(MEAS_INT, t);
            }
            else if (dt > RELOCK_MAX_S) { abortRun("no lock with the internal clock"); }
            break;
        default:
            break;
        }
    }

private:
    enum State { IDLE, SETTLE_REF, MEAS_REF, RELOCK, MEAS_INT };
    State state = IDLE;
    SourceClockCalInterface* src = nullptr;
    double t0 = 0.0, lastSample = 0.0, lockT = -1.0;
    double refHz = 0.0, errRefHz = 0.0, oldPpm = 0.0;
    bool switched = false;
    std::vector<double> samples;

    static double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

    void enter(State s, double t) {
        state = s;
        t0 = t;
        lastSample = 0.0;
        lockT = -1.0;
        samples.clear();
    }
    // one value per 100 ms (the snapshot is refreshed slower than the GUI frames)
    void sample(double errHz, double t) {
        if (t - lastSample < 0.1) { return; }
        lastSample = t;
        samples.push_back(errHz);
    }
    bool stats(double& mean, double& sd) {
        if (samples.size() < 20) { return false; }
        double s = 0.0, q = 0.0;
        for (double v : samples) { s += v; }
        mean = s / samples.size();
        for (double v : samples) { q += (v - mean) * (v - mean); }
        sd = std::sqrt(q / samples.size());
        return true;
    }
    void finish(double errIntHz) {
        // carrier too high by e ppm on the internal clock -> it runs slow by e ppm
        double e = (errIntHz - errRefHz) / refHz * 1e6;
        double ppm = std::round((oldPpm - e) * 1000.0) / 1000.0;   // 1 ppb steps
        if (std::fabs(ppm) > MAX_PPM) { abortRun("implausible result (more than 20 ppm)"); return; }
        src->clockCalStore(ppm);
        bool back = src->clockCalUseInternal(false);
        switched = false;
        state = IDLE;
        char b[200];
        snprintf(b, sizeof(b), "done: internal clock %+.0f ppb (was %+.0f ppb, difference %+.1f Hz)%s",
                 ppm * 1000.0, oldPpm * 1000.0, errIntHz - errRefHz, back ? "" : " - switching back to the reference FAILED");
        src->clockCalStatus(b, false);
    }
    void abortRun(const char* why) {
        bool back = true;
        if (switched && src) { back = src->clockCalUseInternal(false); }
        switched = false;
        state = IDLE;
        if (src) {
            std::string s = std::string("aborted: ") + why + (back ? "" : " - switching back to the reference FAILED");
            src->clockCalStatus(s, false);
        }
    }
};
