// Unit test of the calibration of an SDR's internal clock against its reference
// (decoder_modules/ch_tetra_demodulator/src/clock_cal.h) with a simulated
// source and a simulated carrier measurement.
//
// Build and run (from the repository root; test_tetra_schwartz.py does this):
//   g++ -std=c++17 -DTETRA_CLOCK_CAL_STANDALONE -Icore/src -Idecoder_modules/ch_tetra_demodulator/src e2e/gen/test_clock_cal.cpp -o test_clock_cal
//   ./test_clock_cal

#include <cstdio>
#include <cmath>
#include <string>
#include "clock_cal.h"

static int failures = 0;
#define CHECK(cond, what)                                                   \
    do {                                                                    \
        if (!(cond)) { printf("FAIL: %s (line %d)\n", what, __LINE__); failures++; } \
        else { printf("ok:   %s\n", what); }                               \
    } while (0)

// simulated USRP: reference or internal clock with a true oscillator error
struct FakeSource : SourceClockCalInterface {
    bool request = false, cancel = false, possible = true, internal = false, failSwitch = false;
    double ppm = 0.25;           // correction stored for the internal clock
    double truePpm = -1.40;      // real error of the internal oscillator (positive = fast)
    int stores = 0;
    std::string status;
    bool busy = false;
    std::string clockCalSourceName() override { return "USRP"; }
    bool clockCalTakeRequest() override { bool r = request; request = false; return r; }
    bool clockCalTakeCancel() override { bool c = cancel; cancel = false; return c; }
    bool clockCalPossible() override { return possible; }
    bool clockCalUseInternal(bool i) override { if (failSwitch && i) { return false; } internal = i; return true; }
    double clockCalPpm() override { return ppm; }
    void clockCalStore(double v) override { ppm = v; stores++; }
    void clockCalStatus(const std::string& t, bool b) override { status = t; busy = b; }
};

static const double F = 438.2375e6;      // DL frequency (SYSINFO)
static const double CELL_ERR = 12.0;     // the cell's own error: must cancel out

// carrier error as the TETRA demodulator measures it, with a little noise
static double measured(const FakeSource& s, int k) {
    double e = CELL_ERR + ((k % 7) - 3) * 0.3;
    if (s.internal) { e += -(s.truePpm - s.ppm) * F * 1e-6; }
    return e;
}

// run the state machine; lockLossAt: seconds after the start at which the lock
// is lost for good on the internal clock (-1: never); relockAfter: seconds the
// decoder needs after a clock switch
static double run(TetraClockCal& c, FakeSource& s, double maxS, double lockLossAt = -1.0, double relockAfter = 1.5) {
    double t = 1000.0, t0 = t, switchT = -1.0;
    bool wasInternal = false;
    for (int k = 0; t - t0 < maxS; k++, t += 0.02) {
        if (s.internal && !wasInternal) { switchT = t; }
        wasInternal = s.internal;
        bool locked = !(s.internal && switchT >= 0.0 && t - switchT < relockAfter);
        if (lockLossAt >= 0.0 && s.internal && t - t0 > lockLossAt) { locked = false; }
        c.step(&s, true, locked, measured(s, k), F, t);
        if (!c.busy() && !s.request && t - t0 > 0.1) { break; }
    }
    return t - t0;
}

int main() {
    // 1. normal calibration: result = true oscillator error, cell error cancels
    {
        TetraClockCal c;
        FakeSource s;
        s.request = true;
        run(c, s, 120.0);
        CHECK(s.stores == 1, "calibration stored once");
        CHECK(std::fabs(s.ppm - s.truePpm) <= 0.002, "result = internal oscillator error (cell error cancels)");
        CHECK(!s.internal, "switched back to the reference");
        CHECK(!s.busy && s.status.rfind("done", 0) == 0, "status: done");
        printf("      %s\n", s.status.c_str());
    }
    // 2. lock lost on the internal clock: abort, nothing stored, back on the reference
    {
        TetraClockCal c;
        FakeSource s;
        s.request = true;
        double old = s.ppm;
        run(c, s, 120.0, 9.0);
        CHECK(s.stores == 0 && s.ppm == old, "lock lost: nothing stored");
        CHECK(!s.internal, "lock lost: switched back to the reference");
        CHECK(s.status.rfind("aborted", 0) == 0, "lock lost: status aborted");
    }
    // 3. no relock after the switch
    {
        TetraClockCal c;
        FakeSource s;
        s.request = true;
        run(c, s, 120.0, -1.0, 1000.0);
        CHECK(s.stores == 0 && !s.internal && s.status.find("no lock") != std::string::npos, "no relock within the time limit: aborted, back on the reference");
    }
    // 4. cancel while measuring on the internal clock
    {
        TetraClockCal c;
        FakeSource s;
        s.request = true;
        double t = 0.0;
        for (int k = 0; k < 600; k++, t += 0.02) {   // 12 s: on the internal clock by then
            c.step(&s, true, true, measured(s, k), F, t);
        }
        bool wasInternal = s.internal;
        s.cancel = true;
        c.step(&s, true, true, measured(s, 0), F, t);
        CHECK(wasInternal && !s.internal && s.stores == 0 && !c.busy(), "cancel: back on the reference, nothing stored");
    }
    // 5. not on a reference / not locked: refused at once
    {
        TetraClockCal c;
        FakeSource s;
        s.request = true;
        s.possible = false;
        c.step(&s, true, true, 0.0, F, 0.0);
        CHECK(!c.busy() && s.status.rfind("failed", 0) == 0, "no reference selected: refused");
        s.request = true;
        s.possible = true;
        c.step(&s, true, false, 0.0, F, 0.1);
        CHECK(!c.busy() && s.status.rfind("failed", 0) == 0, "decoder not locked: refused");
        s.request = true;
        c.step(&s, false, true, 0.0, F, 0.2);
        CHECK(!c.busy() && s.status.rfind("failed", 0) == 0, "TETRA demodulator not running: refused");
    }
    // 6. switching to the internal clock fails
    {
        TetraClockCal c;
        FakeSource s;
        s.request = true;
        s.failSwitch = true;
        run(c, s, 60.0);
        CHECK(s.stores == 0 && !c.busy() && s.status.rfind("aborted", 0) == 0, "switch failure: aborted");
    }

    printf("%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
