#pragma once
// Maths of the TETRA TX analysis (no SDR++ dependencies, unit-testable).
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

struct TxMath {
    static constexpr double PI = 3.14159265358979323846;
    static constexpr double RS = 18000.0;     // symbol rate
    static constexpr double ALPHA = 0.35;     // roll-off
    static constexpr double ACP_OFF[6] = { -75000.0, -50000.0, -25000.0, 25000.0, 50000.0, 75000.0 };

    // TETRA filter power weighting (|RRC|^2 = raised cosine), f relative to the channel centre
    static double rcWeight(double f) {
        double a = std::fabs(f);
        double f1 = (1.0 - ALPHA) * RS / 2.0, f2 = (1.0 + ALPHA) * RS / 2.0;
        if (a <= f1) { return 1.0; }
        if (a >= f2) { return 0.0; }
        return 0.5 * (1.0 + std::cos(PI * (a - f1) / (ALPHA * RS)));
    }

    static void fft(std::vector<std::complex<float>>& a) {
        const size_t n = a.size();
        for (size_t i = 1, j = 0; i < n; i++) {
            size_t bit = n >> 1;
            for (; j & bit; bit >>= 1) { j ^= bit; }
            j ^= bit;
            if (i < j) { std::swap(a[i], a[j]); }
        }
        for (size_t len = 2; len <= n; len <<= 1) {
            double ang = -2.0 * PI / (double)len;
            std::complex<float> wl((float)std::cos(ang), (float)std::sin(ang));
            for (size_t i = 0; i < n; i += len) {
                std::complex<double> w(1.0, 0.0);
                const std::complex<double> wld(std::cos(ang), std::sin(ang));
                for (size_t j = 0; j < len / 2; j++) {
                    std::complex<float> wf((float)w.real(), (float)w.imag());
                    std::complex<float> u = a[i + j], v = a[i + j + len / 2] * wf;
                    a[i + j] = u + v;
                    a[i + j + len / 2] = u - v;
                    w *= wld;
                }
            }
            (void)wl;
        }
    }

    // Welch spectrum of 'x' (per-bin power, fft-shifted, sums to the mean power)
    static void welch(const std::vector<std::complex<float>>& x, int n, std::vector<double>& out, double& enbwBins) {
        std::vector<float> w(n);
        double sw = 0.0, sw2 = 0.0;
        for (int i = 0; i < n; i++) {
            double t = 2.0 * PI * i / (n - 1);
            w[i] = (float)(0.35875 - 0.48829 * std::cos(t) + 0.14128 * std::cos(2 * t) - 0.01168 * std::cos(3 * t));
            sw += w[i];
            sw2 += (double)w[i] * w[i];
        }
        enbwBins = n * sw2 / (sw * sw);
        out.assign(n, 0.0);
        int hop = n / 2, segs = 0;
        std::vector<std::complex<float>> buf(n);
        for (size_t s = 0; s + n <= x.size(); s += hop) {
            for (int i = 0; i < n; i++) { buf[i] = x[s + i] * w[i]; }
            fft(buf);
            for (int i = 0; i < n; i++) { out[i] += std::norm(buf[i]); }
            segs++;
        }
        if (!segs) { return; }
        double norm = 1.0 / ((double)segs * n * sw2);
        std::vector<double> sh(n);
        for (int i = 0; i < n; i++) { sh[(i + n / 2) % n] = out[i] * norm; }
        out.swap(sh);
    }

    struct Metrics {
        double chan = 0, acp[6] = { 0 }, acpFloor[6] = { 0 };
        double obw = 0, bw26 = 0, shLo = 0, shHi = 0, shFloor = 0, slLo = 0, slHi = 0;
        double inband = 0, noise = 0;
    };

    // p: per-bin power (fft-shifted), floor: optional reference (same layout)
    static Metrics evaluate(const std::vector<double>& p, const std::vector<double>* floorP, double fs, double carrierHz) {
        Metrics m;
        const int n = (int)p.size();
        const double bin = fs / n;
        auto fOf = [&](int k) { return (k - n / 2) * bin; };
        auto kOf = [&](double f) { return (int)std::lround(f / bin) + n / 2; };
        auto bandMean = [&](const std::vector<double>& v, double fa, double fb) {
            int ka = (std::max)(0, kOf(fa)), kb = (std::min)(n - 1, kOf(fb));
            double s = 0; int c = 0;
            for (int k = ka; k <= kb; k++) { s += v[k]; c++; }
            return c ? s / c : 0.0;
        };
        // noise density far from the carrier (both sides, 95..115 kHz)
        double noise = 0.5 * (bandMean(p, -115000, -95000) + bandMean(p, 95000, 115000));
        m.noise = noise;
        auto weighted = [&](const std::vector<double>& v, double off) {
            double s = 0;
            for (int k = 0; k < n; k++) {
                double wgt = rcWeight(fOf(k) - off);
                if (wgt > 0) { s += v[k] * wgt; }
            }
            return s;
        };
        double wsum = 0;
        for (int k = 0; k < n; k++) { wsum += rcWeight(fOf(k)); }
        m.chan = weighted(p, 0.0);
        double chan = m.chan > 1e-30 ? m.chan : 1e-30;
        for (int i = 0; i < 6; i++) {
            double a = weighted(p, ACP_OFF[i]);
            m.acp[i] = 10.0 * std::log10((std::max)(a, 1e-30) / chan);
            double fl = noise * wsum;
            if (floorP && (int)floorP->size() == n) { fl = (std::max)(fl, weighted(*floorP, ACP_OFF[i])); }
            m.acpFloor[i] = 10.0 * std::log10((std::max)(fl, 1e-30) / chan);
        }
        // occupied bandwidth 99 % (noise removed), within +-60 kHz
        {
            int ka = kOf(-60000), kb = kOf(60000);
            std::vector<double> c;
            double tot = 0;
            for (int k = ka; k <= kb; k++) { tot += (std::max)(0.0, p[k] - noise); c.push_back(tot); }
            if (tot > 0) {
                int lo = 0, hi = (int)c.size() - 1;
                while (lo < (int)c.size() && c[lo] < 0.005 * tot) { lo++; }
                while (hi > 0 && c[hi] > 0.995 * tot) { hi--; }
                m.obw = (hi - lo + 1) * bin;
            }
        }
        // in-band density (|f| 1..5 kHz, avoids the carrier line), smoothed spectrum
        double inb = 0.5 * (bandMean(p, -5000, -1000) + bandMean(p, 1000, 5000));
        m.inband = inb;
        int sm = (std::max)(1, (int)std::lround(400.0 / bin));
        std::vector<double> s(n, 0.0);
        {
            double acc = 0; int cnt = 0;
            for (int k = 0; k < n; k++) {
                acc += p[k]; cnt++;
                if (k >= sm) { acc -= p[k - sm]; cnt--; }
                s[(std::max)(0, k - sm / 2)] = acc / cnt;
            }
        }
        // -26 dB bandwidth (relative to the in-band density)
        {
            double th = inb * std::pow(10.0, -2.6);
            int kc = n / 2, lo = kc, hi = kc;
            for (int k = kOf(-50000); k < kc; k++) { if (s[k] >= th) { lo = k; break; } }
            for (int k = kOf(50000); k > kc; k--) { if (s[k] >= th) { hi = k; break; } }
            m.bw26 = (hi - lo) * bin;
        }
        // shoulders: mean density 13.5..20 kHz from the centre, relative to in-band
        {
            double lo = bandMean(p, -20000, -13500), hi = bandMean(p, 13500, 20000);
            m.shLo = 10.0 * std::log10((std::max)(lo, 1e-30) / (std::max)(inb, 1e-30));
            m.shHi = 10.0 * std::log10((std::max)(hi, 1e-30) / (std::max)(inb, 1e-30));
            double fl = noise;
            if (floorP && (int)floorP->size() == n) {
                fl = (std::max)(fl, 0.5 * (bandMean(*floorP, -20000, -13500) + bandMean(*floorP, 13500, 20000)));
            }
            m.shFloor = 10.0 * std::log10((std::max)(fl, 1e-30) / (std::max)(inb, 1e-30));
        }
        // edge slope between -6 dB and -30 dB (if the -30 dB point is above the floor)
        {
            double t6 = inb * std::pow(10.0, -0.6), t30 = inb * std::pow(10.0, -3.0);
            bool reach = t30 > 2.0 * noise;
            if (reach) {
                int kc = n / 2;
                auto edge = [&](int dir, double th) {
                    for (int k = kc + dir * kOf(1000) - dir * n / 2; k > 0 && k < n - 1; k += dir) {
                        if (s[k] < th) { return std::fabs(fOf(k)); }
                    }
                    return 0.0;
                };
                double l6 = edge(-1, t6), l30 = edge(-1, t30), h6 = edge(1, t6), h30 = edge(1, t30);
                if (l30 > l6 + bin) { m.slLo = 24.0 / ((l30 - l6) / 1000.0); }
                if (h30 > h6 + bin) { m.slHi = 24.0 / ((h30 - h6) / 1000.0); }
            }
        }
        return m;
    }

    // Carrier line (transmitter LO leakage) from a long FFT (fine resolution, so
    // the modulation spreads over many bins while the line stays in a few).
    // p: per-bin power of the long FFT (fft-shifted), chan: channel power in the
    // same normalisation, nAvg: number of averaged spectra (detection threshold).
    static void carrierLeak(const std::vector<double>& p, double fs, double carrierHz, double chan, int nAvg,
                            double& dbc, bool& below) {
        const int n = (int)p.size();
        const double bin = fs / n;
        int kc = (int)std::lround(carrierHz / bin) + n / 2;
        const int hw = 4;   // Blackman-Harris main lobe
        dbc = 0.0; below = true;
        if (n < 64 || kc - hw < 1 || kc + hw >= n - 1 || chan <= 0) { return; }
        double line = 0;
        for (int k = kc - hw; k <= kc + hw; k++) { line += p[k]; }
        auto mean = [&](double fa, double fb) {
            int ka = (int)std::lround(fa / bin) + n / 2, kb = (int)std::lround(fb / bin) + n / 2;
            ka = (std::max)(0, ka); kb = (std::min)(n - 1, kb);
            double s = 0; int c = 0;
            for (int k = ka; k <= kb; k++) { s += p[k]; c++; }
            return c ? s / c : 0.0;
        };
        double around = 0.5 * (mean(carrierHz - 1500, carrierHz - 200) + mean(carrierHz + 200, carrierHz + 1500));
        double expect = (2 * hw + 1) * around;
        // statistical detection threshold: 3 sigma of the averaged modulation power
        double thr = 3.0 * expect / std::sqrt((double)(2 * hw + 1) * (std::max)(1, nAvg));
        double excess = line - expect;
        if (excess > thr) { dbc = 10.0 * std::log10(excess / chan); below = false; }
        else { dbc = 10.0 * std::log10((std::max)(thr, 1e-30) / chan); below = true; }
    }

    // ideal TETRA spectrum figures (for comparison)
    static void idealFigures(double& obw, double& bw26, double& slope) {
        const double step = 5.0;
        std::vector<double> c;
        double tot = 0;
        for (double f = -15000; f <= 15000; f += step) { tot += rcWeight(f); c.push_back(tot); }
        int lo = 0, hi = (int)c.size() - 1;
        while (c[lo] < 0.005 * tot) { lo++; }
        while (c[hi] > 0.995 * tot) { hi--; }
        obw = (hi - lo + 1) * step;
        double f26 = 0, f6 = 0, f30 = 0;
        for (double f = 0; f < 15000; f += 1.0) {
            double w = rcWeight(f);
            if (!f6 && w < std::pow(10.0, -0.6)) { f6 = f; }
            if (!f26 && w < std::pow(10.0, -2.6)) { f26 = f; }
            if (!f30 && w < std::pow(10.0, -3.0)) { f30 = f; }
        }
        bw26 = 2.0 * f26;
        slope = (f30 > f6) ? 24.0 / ((f30 - f6) / 1000.0) : 0.0;
    }

    // Snap to carrier (see TetraBandMonitor::snapCarrier); true: found
    static bool findCarrier(const std::vector<std::complex<float>>& lastX, double fs, double center, double clickOffsetHz, double& outOffsetHz) {
        const int n = (int)(std::min)(lastX.size(), (size_t)(std::max)(4096.0, fs * 0.01));
        const double step = 500.0, span = 45000.0;
        const int nf = (int)(2.0 * span / step) + 1;
        std::vector<float> win(n);
        for (int i = 0; i < n; i++) { win[i] = 0.5f - 0.5f * std::cos(2.0f * (float)PI * (float)i / (float)(n - 1)); }
        std::vector<double> p(nf);
        for (int k = 0; k < nf; k++) {
            double f = clickOffsetHz - span + k * step;
            if (std::fabs(f) > fs / 2.0) { p[k] = 0.0; continue; }
            // phasor recursion in double, renormalised every 1024 samples
            std::complex<double> rot = std::polar(1.0, -2.0 * PI * f / fs), ph(1.0, 0.0), acc(0.0, 0.0);
            for (int i = 0; i < n; i++) {
                acc += std::complex<double>(lastX[i].real() * win[i], lastX[i].imag() * win[i]) * ph;
                ph *= rot;
                if ((i & 1023) == 1023) { ph /= std::abs(ph); }
            }
            p[k] = std::norm(acc);
        }
        // 18 kHz window with the most power, centre within +-25 kHz of the click
        const int half = (int)(9000.0 / step), reach = (int)(25000.0 / step), c0 = nf / 2;
        int best = -1;
        double bestSum = -1.0;
        for (int c = c0 - reach; c <= c0 + reach; c++) {
            if (c - half < 0 || c + half >= nf) { continue; }
            double sum = 0.0;
            for (int k = c - half; k <= c + half; k++) { sum += p[k]; }
            if (sum > bestSum) { bestSum = sum; best = c; }
        }
        if (best < 0) { return false; }
        // floor: lower third of the whole evaluated span (mostly outside the carrier)
        std::vector<double> sorted(p);
        std::nth_element(sorted.begin(), sorted.begin() + nf / 3, sorted.end());
        double floor = (std::max)(sorted[nf / 3], 1e-30);   // noise: about 0.4 x the mean
        double chan = bestSum / (2 * half + 1);
        if (10.0 * std::log10(chan / floor) < 8.0) { return false; }
        // power centroid over +-12.5 kHz, values above the floor only
        double sw = 0.0, sf = 0.0;
        const int h2 = (int)(12500.0 / step);
        for (int k = (std::max)(0, best - h2); k <= (std::min)(nf - 1, best + h2); k++) {
            double w = (std::max)(0.0, p[k] - 2.0 * floor);
            sw += w;
            sf += w * (clickOffsetHz - span + k * step);
        }
        if (!(sw > 0.0)) { return false; }
        double fOff = sf / sw;
        // the skirt of a carrier further away pulls the window to the edge of the
        // search range: the centre itself has to be within +-25 kHz of the click
        if (std::fabs(fOff - clickOffsetHz) > 25000.0) { return false; }
        double fAbs = center + fOff;
        double r = std::round(fAbs / 6250.0) * 6250.0;
        if (std::fabs(r - fAbs) <= 1000.0) { fAbs = r; }
        else { fAbs = std::round(fAbs / 10.0) * 10.0; }
        outOffsetHz = fAbs - center;
        return true;
    }

};
