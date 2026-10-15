#pragma once
// Signal processing of the burst analysis (no GUI / SDR++ dependencies, so it
// can be tested on its own): burst detection on the power envelope, averaged
// burst profile, and the figures derived from the averaged profile (the
// modulation of the single bursts averages out).

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <deque>
#include <vector>

struct TetraBurstResult {
    int state = 0;                 // 0 no data, 1 no bursts (no transmission or continuous), 2 bursts
    bool continuous = false;
    float floorDbfs = -200.0f, topDbfs = -200.0f;
    uint64_t nFull = 0, nHalf = 0;
    float burstsPerSec = 0.0f;
    // figures of the averaged full-slot profile (half-slot if no full-slot bursts yet)
    bool fromHalf = false;
    int averaged = 0;
    float powerDbfs = -200.0f;
    float riseSym = NAN, fallSym = NAN;    // -20 dBc -> -1 dBc / -1 dBc -> -20 dBc
    float lengthSym = NAN;                 // between the -3 dB points
    float ripplePosDb = NAN, rippleNegDb = NAN;
    float offBeforeDbc = NAN, offAfterDbc = NAN;
    float floorDbc = NAN;                  // measurement floor relative to the burst
    std::vector<float> profileDbc;         // averaged profile, 1 point per sample, from -PRE
    double fs = 144000.0;
    double preS = 0.004;
};

class TetraBurstDsp {
public:
    static constexpr double FS = 144000.0;
    static constexpr double SYM = 18000.0;
    static constexpr double PRE = 0.004, POST = 0.017;

    void reset() {
        for (int c = 0; c < 2; c++) { prof[c].clear(); pbAvg[c] = 0.0; nAvg[c] = 0; }
        acc = TetraBurstResult();
        recent.clear();
        processedValid = false;
    }

    const TetraBurstResult& result() const { return acc; }

    // x: the newest samples (x[0] has the absolute index 'base'), nowS: a
    // monotonic time in seconds. Windows may overlap; every burst is taken once.
    void process(const std::vector<std::complex<float>>& x, uint64_t base, double nowS, int avgN) {
        const size_t W = x.size();
        const size_t SMOOTH = (size_t)(FS / SYM);           // one symbol
        const size_t preN = (size_t)(FS * PRE), postN = (size_t)(FS * POST);
        if (W < preN + postN + 4 * SMOOTH) { return; }
        if (!processedValid) { processed = base; processedValid = true; }
        std::vector<double> p(W), ps(W);
        for (size_t i = 0; i < W; i++) { p[i] = (double)std::norm(x[i]); }
        double run = 0.0;
        for (size_t i = 0; i < W; i++) {
            run += p[i];
            if (i >= SMOOTH) { run -= p[i - SMOOTH]; }
            ps[i] = run / (double)(std::min)(SMOOTH, i + 1);
        }
        // floor (10th percentile) and top (99.5th percentile)
        std::vector<double> srt(ps.begin() + SMOOTH, ps.end());
        size_t k10 = srt.size() / 10, k995 = (size_t)(srt.size() * 0.995);
        std::nth_element(srt.begin(), srt.begin() + k10, srt.end());
        double fl = srt[k10];
        std::nth_element(srt.begin(), srt.begin() + k995, srt.end());
        double top = srt[k995];
        double flDb = 10.0 * std::log10((std::max)(fl, 1e-30)), topDb = 10.0 * std::log10((std::max)(top, 1e-30));
        acc.floorDbfs = (float)flDb;
        acc.topDbfs = (float)topDb;
        if (acc.state == 0) { acc.state = 1; }
        acc.continuous = (topDb - flDb) < 10.0;
        if (topDb - flDb >= 12.0) {
            double thr = std::pow(10.0, 0.1 * (flDb + 0.5 * (topDb - flDb)));
            size_t i = processed > base ? (size_t)(processed - base) : 0;
            i = (std::max)(i, preN + SMOOTH);
            const size_t quietN = (size_t)(FS * 0.001);
            for (; i + postN < W; i++) {
                if (!(ps[i - 1] < thr && ps[i] >= thr)) { continue; }
                bool quiet = true;
                for (size_t k = i - quietN; k < i; k++) { if (ps[k] >= thr) { quiet = false; break; } }
                size_t j = i;
                while (j < W && ps[j] >= thr) { j++; }
                if (j >= W) { break; }
                double d = (double)(j - i) / FS;
                if (quiet && d >= 0.003 && d <= 0.016) { addBurst(ps, p, i, j, fl, d > 0.009 ? 0 : 1, avgN, nowS); }
                i = j;
            }
        }
        processed = base + (W - postN);
        while (!recent.empty() && nowS - recent.front() > 5.0) { recent.pop_front(); }
        acc.burstsPerSec = (float)recent.size() / 5.0f;
        if (acc.nFull + acc.nHalf > 0) { acc.state = 2; }
        int c = prof[0].empty() ? 1 : 0;
        acc.fromHalf = c == 1;
        if (!prof[c].empty()) { evaluate(c, fl); }
        acc.fs = FS;
        acc.preS = PRE;
    }

private:
    // one burst between the threshold crossings i (rising) and j (falling)
    void addBurst(const std::vector<double>& ps, const std::vector<double>& p, size_t i, size_t j, double fl, int cls, int avgN, double nowS) {
        const size_t W = ps.size();
        const size_t ms05 = (size_t)(FS * 0.0005), ms1 = (size_t)(FS * 0.001);
        const size_t preN = (size_t)(FS * PRE), profN = preN + (size_t)(FS * POST);
        if (j <= i + 2 * ms05 + 8) { return; }
        double sum = 0.0;
        for (size_t k = i + ms05; k < j - ms05; k++) { sum += p[k]; }
        double pb = sum / (double)((j - ms05) - (i + ms05));
        if (!(pb > fl * 10.0)) { return; }
        // align on the -3 dB point of the rising edge
        size_t lo = i > ms1 ? i - ms1 : 0;
        size_t a50 = lo;
        while (a50 < j && ps[a50] < 0.5 * pb) { a50++; }
        if (a50 < preN || a50 - preN + profN > W) { return; }
        int& n = nAvg[cls];
        n = (std::min)(n + 1, (std::max)(1, avgN));
        double a = 1.0 / n;
        std::vector<double>& pr = prof[cls];
        if (pr.size() != profN) { pr.assign(profN, 0.0); n = 1; a = 1.0; }
        for (size_t k = 0; k < profN; k++) { pr[k] += a * (ps[a50 - preN + k] / pb - pr[k]); }
        pbAvg[cls] = n == 1 ? pb : pbAvg[cls] + a * (pb - pbAvg[cls]);
        if (cls == 0) { acc.nFull++; } else { acc.nHalf++; }
        recent.push_back(nowS);
    }

    // figures from the averaged profile of class c
    void evaluate(int c, double floorLin) {
        const std::vector<double>& pr = prof[c];
        const size_t n = pr.size();
        const size_t preN = (size_t)(FS * PRE), sym = (size_t)(FS / SYM);
        const size_t ms15 = (size_t)(FS * 0.0015), ms3 = (size_t)(FS * 0.003), ms05 = (size_t)(FS * 0.0005);
        acc.averaged = nAvg[c];
        double pb = pbAvg[c];
        acc.powerDbfs = (float)(10.0 * std::log10((std::max)(pb, 1e-30)));
        // -3 dB points (profile is normalised to the burst power)
        size_t a = preN > sym ? preN - sym : 0;
        while (a < n && pr[a] < 0.5) { a++; }
        size_t b = n - 1;
        while (b > a && pr[b] < 0.5) { b--; }
        if (b <= a + 4 * sym) { return; }
        // reference: mean of the useful part
        double ref = 0.0;
        size_t u0 = a + ms05, u1 = b > ms05 ? b - ms05 : b;
        if (u1 <= u0) { return; }
        for (size_t k = u0; k < u1; k++) { ref += pr[k]; }
        ref /= (double)(u1 - u0);
        auto at = [&](size_t k) { return pr[k] / ref; };
        // -3 dB length with linear interpolation
        auto cross = [&](size_t k0, size_t k1, double lvl) {   // crossing between k0 and k1 (adjacent)
            double y0 = at(k0), y1 = at(k1);
            return (y1 == y0) ? (double)k0 : (double)k0 + (lvl - y0) / (y1 - y0) * ((double)k1 - (double)k0);
        };
        double xa = a > 0 ? cross(a - 1, a, 0.5) : (double)a;
        double xb = b + 1 < n ? cross(b, b + 1, 0.5) : (double)b;
        acc.lengthSym = (float)((xb - xa) / FS * SYM);
        bool floorOk = floorLin < 0.003 * pb;   // floor at least 25 dB below the burst
        // ramp up: last sample below -20 dBc before the edge, first above -1 dBc
        size_t r90 = a; while (r90 < b && at(r90) < 0.794) { r90++; }
        size_t r10 = a; while (r10 > 0 && at(r10) >= 0.01) { r10--; }
        size_t f90 = b; while (f90 > a && at(f90) < 0.794) { f90--; }
        size_t f10 = b; while (f10 + 1 < n && at(f10) >= 0.01) { f10++; }
        acc.riseSym = floorOk ? (float)((double)(r90 - r10) / FS * SYM) : NAN;
        acc.fallSym = floorOk ? (float)((double)(f10 - f90) / FS * SYM) : NAN;
        // ripple of the useful part, one symbol away from the -1 dB points
        if (f90 > r90 + 4 * sym) {
            double mx = 0.0, mn = 1e300;
            for (size_t k = r90 + sym; k < f90 - sym; k++) { mx = (std::max)(mx, at(k)); mn = (std::min)(mn, at(k)); }
            acc.ripplePosDb = (float)(10.0 * std::log10(mx));
            acc.rippleNegDb = (float)(10.0 * std::log10((std::max)(mn, 1e-30)));
        }
        // power outside: 3 .. 1.5 ms before the rising and after the falling -3 dB point
        size_t ia = (size_t)xa, ib = (size_t)xb;
        if (ia >= ms3) {
            double s = 0.0;
            for (size_t k = ia - ms3; k < ia - ms15; k++) { s += at(k); }
            acc.offBeforeDbc = (float)(10.0 * std::log10((std::max)(s / (double)(ms3 - ms15), 1e-30)));
        }
        if (ib + ms3 < n) {
            double s = 0.0;
            for (size_t k = ib + ms15; k < ib + ms3; k++) { s += at(k); }
            acc.offAfterDbc = (float)(10.0 * std::log10((std::max)(s / (double)(ms3 - ms15), 1e-30)));
        }
        acc.floorDbc = (float)(10.0 * std::log10((std::max)(floorLin, 1e-30) / (pb * ref)));
        acc.profileDbc.resize(n);
        for (size_t k = 0; k < n; k++) { acc.profileDbc[k] = (float)(10.0 * std::log10((std::max)(at(k), 1e-12))); }
    }

    std::vector<double> prof[2];   // 0 full slot, 1 half slot (linear, normalised per burst)
    double pbAvg[2] = { 0.0, 0.0 };
    int nAvg[2] = { 0, 0 };
    TetraBurstResult acc;
    std::deque<double> recent;
    uint64_t processed = 0;
    bool processedValid = false;
};
