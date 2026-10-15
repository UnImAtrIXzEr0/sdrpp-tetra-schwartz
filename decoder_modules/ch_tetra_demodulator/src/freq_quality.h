#pragma once
// Evaluations of the trend data (no GUI): frequency stability (overlapping
// Allan deviation of the carrier vs. SYSINFO frequency) and reception
// quality versus level (CRC errors / MER per 1 dB level bin).

#include <algorithm>
#include <cmath>
#include <vector>
#include "trend.h"

namespace tetra_eval {
    struct AdevPoint { double tau; double adev; int n; };

    // y: fractional frequency (ppb), one value per second, NaN = gap.
    // Uses the longest run without gaps.
    inline std::vector<AdevPoint> allanDeviation(const std::vector<double>& y, size_t& runLen) {
        size_t bestA = 0, bestL = 0, a = 0;
        for (size_t i = 0; i <= y.size(); i++) {
            if (i == y.size() || std::isnan(y[i])) {
                if (i - a > bestL) { bestL = i - a; bestA = a; }
                a = i + 1;
            }
        }
        runLen = bestL;
        std::vector<AdevPoint> out;
        if (bestL < 10) { return out; }
        // phase (ppb * s)
        std::vector<double> x(bestL + 1, 0.0);
        for (size_t i = 0; i < bestL; i++) { x[i + 1] = x[i] + y[bestA + i]; }
        const size_t N = x.size();
        for (size_t m = 1; 2 * m + 1 < N && m <= N / 3; m *= 2) {
            double s = 0.0;
            size_t cnt = 0;
            for (size_t k = 0; k + 2 * m < N; k++) {
                double d = x[k + 2 * m] - 2.0 * x[k + m] + x[k];
                s += d * d;
                cnt++;
            }
            if (!cnt) { break; }
            double avar = s / (2.0 * (double)m * (double)m * (double)cnt);
            out.push_back({ (double)m, std::sqrt(avar), (int)cnt });
        }
        return out;
    }

    struct LevelBin { float level; int n; float crcFail; float mer; int merN; };
    struct QualityCurve {
        std::vector<LevelBin> bins;
        float thresholdLevel = NAN;   // lowest level from which on all bins stay within the CRC limit
        bool calibrated = false;
    };

    inline QualityCurve qualityVsLevel(const std::vector<TetraTrendPoint>& pts, float crcLimitPct, int minSamples = 5) {
        QualityCurve q;
        if (pts.empty()) { return q; }
        float lo = INFINITY, hi = -INFINITY;
        for (auto& p : pts) {
            if (std::isnan(p.level)) { continue; }
            lo = (std::min)(lo, p.level);
            hi = (std::max)(hi, p.level);
            if (p.calibrated) { q.calibrated = true; }
        }
        if (!(lo <= hi)) { return q; }
        int b0 = (int)std::floor(lo), b1 = (int)std::floor(hi);
        if (b1 - b0 > 200) { b0 = b1 - 200; }
        std::vector<double> crcSum(b1 - b0 + 1, 0.0), merSum(b1 - b0 + 1, 0.0);
        std::vector<int> n(b1 - b0 + 1, 0), merN(b1 - b0 + 1, 0);
        for (auto& p : pts) {
            if (std::isnan(p.level) || std::isnan(p.crcFail)) { continue; }
            int b = (int)std::floor(p.level) - b0;
            if (b < 0 || b >= (int)n.size()) { continue; }
            crcSum[b] += p.crcFail;
            n[b]++;
            if (!std::isnan(p.mer)) { merSum[b] += p.mer; merN[b]++; }
        }
        for (size_t b = 0; b < n.size(); b++) {
            if (n[b] < minSamples) { continue; }
            q.bins.push_back({ (float)(b0 + (int)b) + 0.5f, n[b], (float)(crcSum[b] / n[b]), merN[b] ? (float)(merSum[b] / merN[b]) : NAN, merN[b] });
        }
        // threshold: above the highest bin that exceeds the limit
        float worst = -INFINITY;
        bool anyGood = false;
        for (auto& b : q.bins) {
            if (b.crcFail > crcLimitPct) { worst = (std::max)(worst, b.level); }
            else { anyGood = true; }
        }
        if (anyGood) {
            if (worst == -INFINITY) { q.thresholdLevel = q.bins.front().level - 0.5f; }   // never above the limit: at most this
            else if (worst < q.bins.back().level) { q.thresholdLevel = worst + 0.5f; }
        }
        return q;
    }
}
