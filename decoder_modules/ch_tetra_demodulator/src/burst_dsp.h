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
#include "tx_analysis_math.h"

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
    // power between the bursts (uplink noise / interference), NaN = not measurable
    float gapDbfs = NAN;
    // timing of the bursts relative to the downlink slot grid (needs the DL tap)
    bool timingValid = false;
    float timingMeanSym = NAN, timingStdSym = NAN;   // -3 dB point of the rising edge after the slot start
    int timingN = 0;
    struct TimingPoint { double t; float offsetSym; float powerDbfs; };
    std::vector<TimingPoint> timing;                 // last bursts (for the scatter plot)
    // switching transients: channel power at -50, -25, 0, +25, +50 kHz in
    // 1.8 ms windows at the ramp-up, in the middle and at the ramp-down,
    // relative to the channel power in the middle (dBc); [window][offset]
    bool transValid = false;
    int transN = 0;
    float transDbc[3][5] = {};
    float transFloorDbc[5] = {};
    // receiver overload check (needs the DL tap): downlink power during the
    // bursts relative to just before / after them. A continuous downlink must
    // not change; a drop means the strong burst compresses the receiver.
    bool desenseValid = false;
    int desenseN = 0;
    float desenseDb = NAN;   // > 0: downlink weaker during the bursts
};

class TetraBurstDsp {
public:
    static constexpr double FS = 144000.0;
    static constexpr double SYM = 18000.0;
    static constexpr double PRE = 0.004, POST = 0.017;

    static constexpr int TRANS_N = 256;                 // 1.78 ms windows
    static constexpr double TRANS_OFF[5] = { -50000.0, -25000.0, 0.0, 25000.0, 50000.0 };

    void reset() {
        for (int c = 0; c < 2; c++) { prof[c].clear(); pbAvg[c] = 0.0; nAvg[c] = 0; }
        timingPts.clear();
        for (auto& w : transAcc) { for (auto& v : w) { v = 0.0; } }
        for (auto& v : transFloorAcc) { v = 0.0; }
        transCount = 0;
        transFloorCount = 0;
        desenseDbAvg = 0.0;
        desenseCount = 0;
        acc = TetraBurstResult();
        recent.clear();
        processedValid = false;
    }

    const TetraBurstResult& result() const { return acc; }

    // x: the newest channel-filtered samples (x[0] has the absolute index
    // 'base'), nowS: a monotonic time in seconds. Windows may overlap; every
    // burst is taken once. raw: the same samples before the channel filter
    // (wide, for the switching transients; raw[k] belongs to x[k + rawDelay]),
    // slotPhase: downlink slot grid (absolute index mod 2040 of symbol 0) or NaN.
    void process(const std::vector<std::complex<float>>& x, uint64_t base, double nowS, int avgN,
                 const std::vector<std::complex<float>>* raw = nullptr, int rawDelay = 0, double slotPhase = NAN,
                 const std::vector<std::complex<float>>* dl = nullptr) {
        curRaw = raw;
        curDl = (dl && dl->size() == x.size()) ? dl : nullptr;
        curRawDelay = rawDelay;
        curBase = base;
        curSlotPhase = slotPhase;
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
        // power between the bursts: everything more than 1 ms away from the
        // bursts (above the half-way level between floor and peak, in dB)
        {
            const size_t ms1g = (size_t)(FS * 0.001);
            double on = (topDb - flDb >= 12.0) ? std::pow(10.0, 0.1 * (flDb + 0.5 * (topDb - flDb))) : INFINITY;
            std::vector<char> busy(W, 0);
            size_t last = (size_t)-1;
            for (size_t k = 0; k < W; k++) { if (ps[k] >= on) { last = k; } if (last != (size_t)-1 && k - last <= ms1g) { busy[k] = 1; } }
            last = (size_t)-1;
            for (size_t k = W; k-- > 0;) { if (ps[k] >= on) { last = k; } if (last != (size_t)-1 && last - k <= ms1g) { busy[k] = 1; } }
            double sum = 0.0;
            size_t cnt = 0;
            for (size_t k = SMOOTH; k < W; k++) { if (!busy[k]) { sum += p[k]; cnt++; } }
            if (cnt > (size_t)(FS * 0.02)) { acc.gapDbfs = (float)(10.0 * std::log10((std::max)(sum / cnt, 1e-30))); }
            else if (topDb - flDb < 10.0) { acc.gapDbfs = NAN; }   // continuous signal, no gaps
            // floor of the transient windows: a quiet stretch
            if (curRaw && cnt > (size_t)(FS * 0.02)) {
                for (size_t k = SMOOTH; k + TRANS_N < W; k += TRANS_N) {
                    bool quiet = true;
                    for (size_t q = k; q < k + TRANS_N; q++) { if (busy[q]) { quiet = false; break; } }
                    if (!quiet) { continue; }
                    double cp[5];
                    if (!chanPowers(k, cp)) { break; }
                    for (int o = 0; o < 5; o++) { transFloorAcc[o] += cp[o]; }
                    transFloorCount++;
                    break;
                }
            }
        }
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
        // timing statistics over the last 50 bursts
        while (timingPts.size() > 400) { timingPts.erase(timingPts.begin()); }
        acc.timing = timingPts;
        if (!timingPts.empty()) {
            // circular mean (the offset wraps at 255 symbols)
            size_t n0 = timingPts.size() > 50 ? timingPts.size() - 50 : 0;
            double sx = 0, sy = 0;
            for (size_t k = n0; k < timingPts.size(); k++) {
                double a = 2.0 * TxMath::PI * timingPts[k].offsetSym / 255.0;
                sx += std::cos(a); sy += std::sin(a);
            }
            double m = std::atan2(sy, sx) / (2.0 * TxMath::PI) * 255.0;
            if (m < 0) { m += 255.0; }
            double v = 0;
            for (size_t k = n0; k < timingPts.size(); k++) {
                double d = timingPts[k].offsetSym - m;
                while (d > 127.5) { d -= 255.0; }
                while (d < -127.5) { d += 255.0; }
                v += d * d;
            }
            size_t cnt = timingPts.size() - n0;
            acc.timingValid = true;
            acc.timingMeanSym = (float)m;
            acc.timingStdSym = (float)std::sqrt(v / cnt);
            acc.timingN = (int)cnt;
        }
        if (desenseCount > 0) {
            acc.desenseValid = desenseCount >= 3;
            acc.desenseN = desenseCount;
            acc.desenseDb = (float)desenseDbAvg;
        }
        if (transCount > 0) {
            acc.transValid = true;
            acc.transN = transCount;
            double ref = transAcc[1][2] / transCount;
            for (int w = 0; w < 3; w++) {
                for (int o = 0; o < 5; o++) { acc.transDbc[w][o] = (float)(10.0 * std::log10((std::max)(transAcc[w][o] / transCount, 1e-30) / (std::max)(ref, 1e-30))); }
            }
            for (int o = 0; o < 5; o++) {
                acc.transFloorDbc[o] = transFloorCount ? (float)(10.0 * std::log10((std::max)(transFloorAcc[o] / transFloorCount, 1e-30) / (std::max)(ref, 1e-30))) : NAN;
            }
        }
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
        // overload check: downlink power in the useful part of the burst vs.
        // 1..3 ms before and after it (the downlink is continuous)
        if (curDl && i >= 3 * ms1 && j + 3 * ms1 < W) {
            const std::vector<std::complex<float>>& d = *curDl;
            double in = 0.0, out = 0.0;
            size_t nIn = 0, nOut = 0;
            for (size_t k = i + ms05; k < j - ms05; k++) { in += std::norm(d[k]); nIn++; }
            for (size_t k = i - 3 * ms1; k < i - ms1; k++) { out += std::norm(d[k]); nOut++; }
            for (size_t k = j + ms1; k < j + 3 * ms1; k++) { out += std::norm(d[k]); nOut++; }
            if (nIn && nOut && in > 0.0 && out > 0.0) {
                double db = 10.0 * std::log10((out / nOut) / (in / nIn));
                desenseCount = (std::min)(desenseCount + 1, 1000000);
                int nn = (std::min)(desenseCount, 20);
                desenseDbAvg += (db - desenseDbAvg) / nn;
            }
        }
        // timing against the downlink slot grid; the one-symbol moving average
        // delays the envelope by (SMOOTH-1)/2 samples
        if (!std::isnan(curSlotPhase)) {
            double smoothDelay = ((double)(FS / SYM) - 1.0) / 2.0;
            double posAbs = (double)(curBase + a50) - smoothDelay;
            double off = std::fmod(posAbs - curSlotPhase, 2040.0);
            if (off < 0) { off += 2040.0; }
            timingPts.push_back({ nowS, (float)(off / 8.0), (float)(10.0 * std::log10((std::max)(pb, 1e-30))) });
        }
        // switching transients (full-slot bursts): windows centred at the rising
        // and falling -3 dB points and in the middle
        if (curRaw && cls == 0) {
            size_t b50 = (std::min)(W - 1, j + ms1);
            while (b50 > a50 && ps[b50] < 0.5 * pb) { b50--; }
            size_t cen[3] = { a50, (a50 + b50) / 2, b50 };
            double cp[3][5];
            bool ok = true;
            for (int w = 0; w < 3 && ok; w++) {
                if (cen[w] < (size_t)(TRANS_N / 2)) { ok = false; break; }
                ok = chanPowers(cen[w] - TRANS_N / 2, cp[w]);
            }
            if (ok) {
                for (int w = 0; w < 3; w++) { for (int o = 0; o < 5; o++) { transAcc[w][o] += cp[w][o]; } }
                transCount++;
            }
        }
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

    // channel powers (TETRA filter weighting) at TRANS_OFF in a TRANS_N window
    // starting at filtered index k (raw samples)
    bool chanPowers(size_t k, double out[5]) {
        if (!curRaw) { return false; }
        long r0 = (long)k - curRawDelay;
        if (r0 < 0 || r0 + TRANS_N > (long)curRaw->size()) { return false; }
        if (transWin.empty()) {
            transWin.resize(TRANS_N);
            for (int i = 0; i < TRANS_N; i++) {
                double t = 2.0 * TxMath::PI * i / (TRANS_N - 1);   // Blackman-Harris: low leakage into the adjacent channels
                transWin[i] = (float)(0.35875 - 0.48829 * std::cos(t) + 0.14128 * std::cos(2 * t) - 0.01168 * std::cos(3 * t));
            }
        }
        std::vector<std::complex<float>> buf(TRANS_N);
        for (int i = 0; i < TRANS_N; i++) { buf[i] = (*curRaw)[(size_t)(r0 + i)] * transWin[i]; }
        TxMath::fft(buf);
        for (int o = 0; o < 5; o++) {
            double sum = 0.0;
            for (int b = 0; b < TRANS_N; b++) {
                double f = (b < TRANS_N / 2 ? b : b - TRANS_N) * FS / TRANS_N;
                double w = TxMath::rcWeight(f - TRANS_OFF[o]);
                if (w > 0.0) { sum += w * std::norm(buf[b]); }
            }
            out[o] = sum;
        }
        return true;
    }

    const std::vector<std::complex<float>>* curRaw = nullptr;
    const std::vector<std::complex<float>>* curDl = nullptr;
    double desenseDbAvg = 0.0;
    int desenseCount = 0;
    int curRawDelay = 0;
    uint64_t curBase = 0;
    double curSlotPhase = NAN;
    std::vector<TetraBurstResult::TimingPoint> timingPts;
    double transAcc[3][5] = {};
    double transFloorAcc[5] = {};
    int transCount = 0, transFloorCount = 0;
    std::vector<float> transWin;

    std::vector<double> prof[2];   // 0 full slot, 1 half slot (linear, normalised per burst)
    double pbAvg[2] = { 0.0, 0.0 };
    int nAvg[2] = { 0, 0 };
    TetraBurstResult acc;
    std::deque<double> recent;
    uint64_t processed = 0;
    bool processedValid = false;
};
