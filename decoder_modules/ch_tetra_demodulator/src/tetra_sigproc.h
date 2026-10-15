#pragma once
// Small signal processing helpers of the TETRA analysis taps (144 kS/s,
// 8 samples per symbol); no SDR++ dependencies, unit-testable.
//  - ChanFilter: 25 kHz channel filter (complex FIR, windowed sinc)
//  - SlotTiming: downlink slot timing and power delay profile from the
//    normal training sequences of the downlink bursts

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <vector>

namespace tetra_sp {
    static constexpr double PI_D = 3.14159265358979323846;
    static constexpr double FS = 144000.0;
    static constexpr int SPS = 8;                 // samples per symbol
    static constexpr int SLOT = 255 * SPS;        // samples per timeslot

    // complex FIR low pass, keeps its state between calls
    class ChanFilter {
    public:
        explicit ChanFilter(double cutoffHz = 13000.0, int ntaps = 97) {
            taps.resize(ntaps);
            double fc = cutoffHz / FS, sum = 0.0;
            int m = ntaps - 1;
            for (int i = 0; i < ntaps; i++) {
                double x = i - m / 2.0;
                double s = x == 0.0 ? 2.0 * fc : std::sin(2.0 * PI_D * fc * x) / (PI_D * x);
                double w = 0.42 - 0.5 * std::cos(2.0 * PI_D * i / m) + 0.08 * std::cos(4.0 * PI_D * i / m);
                taps[i] = (float)(s * w);
                sum += taps[i];
            }
            for (auto& t : taps) { t = (float)(t / sum); }
            hist.assign(2 * ntaps, std::complex<float>(0.0f, 0.0f));
        }
        int delay() const { return (int)taps.size() / 2; }
        void reset() { std::fill(hist.begin(), hist.end(), std::complex<float>(0.0f, 0.0f)); pos = 0; }
        std::complex<float> step(std::complex<float> x) {
            const int n = (int)taps.size();
            hist[pos] = x;
            hist[pos + n] = x;
            pos = (pos + 1) % n;
            // hist[pos .. pos+n-1] = oldest .. newest
            float re = 0.0f, im = 0.0f;
            const std::complex<float>* h = &hist[pos];
            for (int i = 0; i < n; i++) { re += taps[n - 1 - i] * h[i].real(); im += taps[n - 1 - i] * h[i].imag(); }
            return std::complex<float>(re, im);
        }

    private:
        std::vector<float> taps;
        std::vector<std::complex<float>> hist;
        int pos = 0;
    };

    // pi/4-DQPSK phase change for a dibit (EN 300 392-2: 00 +pi/4, 01 +3pi/4, 11 -3pi/4, 10 -pi/4)
    inline double dphi(int b1, int b2) {
        if (!b1 && !b2) { return PI_D / 4; }
        if (!b1 && b2) { return 3 * PI_D / 4; }
        if (b1 && b2) { return -3 * PI_D / 4; }
        return -PI_D / 4;
    }

    // Downlink normal training sequences 1 and 2 (22 bits), located at
    // symbols 122..132 of the normal downlink burst (symbol 121 precedes them)
    static const uint8_t NTS1[22] = { 1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,1, 0,0 };
    static const uint8_t NTS2[22] = { 0,1, 1,1, 1,0, 1,0, 0,1, 0,0, 0,0, 1,1, 0,1, 1,1, 1,0 };
    static constexpr int NTS_FIRST_SYM = 121;   // the symbol before the first phase change

    // 12 symbol points (relative phase) of a training sequence; mirror = other
    // rotation sense (tolerates a spectrally inverted receiver)
    inline std::vector<std::complex<float>> tsPoints(const uint8_t* bits, bool mirror) {
        std::vector<std::complex<float>> s(12);
        double ph = 0.0;
        s[0] = 1.0f;
        for (int k = 0; k < 11; k++) {
            double d = dphi(bits[2 * k], bits[2 * k + 1]);
            ph += mirror ? -d : d;
            s[k + 1] = std::complex<float>((float)std::cos(ph), (float)std::sin(ph));
        }
        return s;
    }

    struct SlotEstimate {
        bool valid = false;
        double phase = 0.0;      // absolute sample index (mod SLOT) of the centre of symbol 0 of a slot
        double quality = 0.0;    // fold peak / median
        int variant = 0;
        std::vector<float> fold; // folded correlation power, SLOT bins
        int peak = 0;            // bin of the peak
    };

    // Folded correlation of x (x[0] = absolute index 'base') with the training
    // sequences, sampled at the symbol centres (12 taps).
    inline SlotEstimate slotTiming(const std::vector<std::complex<float>>& x, uint64_t base) {
        SlotEstimate best;
        const size_t W = x.size();
        if (W < (size_t)(3 * SLOT)) { return best; }
        const int L = 11 * SPS;
        for (int v = 0; v < 2; v++) {
            auto r1 = tsPoints(NTS1, v == 1), r2 = tsPoints(NTS2, v == 1);
            std::vector<float> fold(SLOT, 0.0f);
            for (size_t m = 0; m + L < W; m++) {
                std::complex<float> c1(0.0f, 0.0f), c2(0.0f, 0.0f);
                for (int k = 0; k < 12; k++) {
                    const std::complex<float>& s = x[m + k * SPS];
                    c1 += s * std::conj(r1[k]);
                    c2 += s * std::conj(r2[k]);
                }
                fold[(size_t)((base + m) % SLOT)] += std::norm(c1) + std::norm(c2);
            }
            int pk = (int)(std::max_element(fold.begin(), fold.end()) - fold.begin());
            std::vector<float> srt(fold);
            std::nth_element(srt.begin(), srt.begin() + SLOT / 2, srt.end());
            double med = (std::max)((double)srt[SLOT / 2], 1e-30);
            double q = fold[pk] / med;
            if (q > best.quality) {
                best.quality = q;
                best.variant = v;
                best.peak = pk;
                // parabolic refinement
                double y0 = fold[(pk + SLOT - 1) % SLOT], y1 = fold[pk], y2 = fold[(pk + 1) % SLOT];
                double den = y0 - 2.0 * y1 + y2;
                double frac = den != 0.0 ? 0.5 * (y0 - y2) / den : 0.0;
                double ph = (double)pk + std::clamp(frac, -0.5, 0.5) - (double)(NTS_FIRST_SYM * SPS);
                ph = std::fmod(ph, (double)SLOT);
                if (ph < 0) { ph += SLOT; }
                best.phase = ph;
                best.fold = std::move(fold);
            }
        }
        best.valid = best.quality > 6.0;   // clear training sequence peak
        return best;
    }

    // power delay profile: the fold around its peak, +-span samples, normalised
    inline std::vector<float> delayProfile(const SlotEstimate& e, int span) {
        std::vector<float> p(2 * span + 1, 0.0f);
        if (e.fold.empty()) { return p; }
        float pk = e.fold[e.peak];
        for (int d = -span; d <= span; d++) { p[d + span] = e.fold[(size_t)((e.peak + d + SLOT) % SLOT)] / (pk > 0 ? pk : 1.0f); }
        return p;
    }

    // Synthetic clean downlink (random data, training sequences at their
    // place, RRC alpha 0.35) through the channel filter: reference for the
    // delay profile without multipath. 'echo' adds a delayed copy (tests).
    inline std::vector<std::complex<float>> syntheticDownlink(int slots, unsigned seed, int echoDelaySamples = 0, float echoGain = 0.0f) {
        const int nsym = slots * 255;
        std::vector<std::complex<double>> sym(nsym);
        uint32_t st = seed ? seed : 1;
        auto rnd = [&]() { st = st * 1664525u + 1013904223u; return (int)((st >> 16) & 1); };
        double ph = 0.0;
        for (int i = 0; i < nsym; i++) {
            int s = i % 255;
            int b1, b2;
            if (s >= NTS_FIRST_SYM + 1 && s <= NTS_FIRST_SYM + 11) {
                const uint8_t* t = ((i / 255) & 1) ? NTS2 : NTS1;
                int k = s - (NTS_FIRST_SYM + 1);
                b1 = t[2 * k]; b2 = t[2 * k + 1];
            }
            else { b1 = rnd(); b2 = rnd(); }
            ph += dphi(b1, b2);
            sym[i] = std::polar(1.0, ph);
        }
        // RRC pulse shaping
        const int span = 6;
        std::vector<double> h(2 * span * SPS + 1);
        const double a = 0.35;
        for (int i = 0; i < (int)h.size(); i++) {
            double t = (double)(i - span * SPS) / SPS;
            double v;
            if (std::fabs(t) < 1e-9) { v = 1.0 - a + 4.0 * a / PI_D; }
            else if (std::fabs(std::fabs(t) - 1.0 / (4.0 * a)) < 1e-9) {
                v = a / std::sqrt(2.0) * ((1 + 2 / PI_D) * std::sin(PI_D / (4 * a)) + (1 - 2 / PI_D) * std::cos(PI_D / (4 * a)));
            }
            else { v = (std::sin(PI_D * t * (1 - a)) + 4 * a * t * std::cos(PI_D * t * (1 + a))) / (PI_D * t * (1 - (4 * a * t) * (4 * a * t))); }
            h[i] = v;
        }
        std::vector<std::complex<double>> up((size_t)nsym * SPS + h.size(), 0.0);
        for (int i = 0; i < nsym; i++) {
            for (size_t k = 0; k < h.size(); k++) { up[(size_t)i * SPS + k] += sym[i] * h[k]; }
        }
        // centre of symbol i is at sample i*SPS + span*SPS
        std::vector<std::complex<float>> out((size_t)nsym * SPS);
        double norm = 0.0;
        for (size_t i = 0; i < out.size(); i++) { norm += std::norm(up[i + span * SPS]); }
        norm = std::sqrt(norm / out.size());
        for (size_t i = 0; i < out.size(); i++) {
            std::complex<double> v = up[i + span * SPS];
            if (echoDelaySamples > 0 && i >= (size_t)echoDelaySamples) { v += (double)echoGain * up[i - echoDelaySamples + span * SPS]; }
            out[i] = std::complex<float>((float)(v.real() / norm * 0.3), (float)(v.imag() / norm * 0.3));
        }
        return out;
    }

    // Least-squares channel estimate from the training sequence of every
    // downlink slot: symbol-spaced taps l = -1..3 for each of the 8 sample
    // phases -> power delay profile with sample resolution from -1 to +3
    // symbols (-55 .. +222 us). Averaged over many slots.
    class ChannelEstimator {
    public:
        static constexpr int LMIN = -1, LMAX = 4;          // taps l = LMIN .. LMAX-1
        static constexpr int NT = LMAX - LMIN;             // 5 taps
        static constexpr int K0 = LMAX - 1, K1 = 11 + LMIN; // equations k = K0 .. K1 (8)
        static constexpr int NE = K1 - K0 + 1;
        static constexpr int NBINS = NT * SPS;             // delay bins, bin 0 = LMIN symbols

        ChannelEstimator() { build(); }
        void reset() { std::fill(acc.begin(), acc.end(), 0.0); slots = 0; avgCorr = 0.0; }
        // exponential averaging: keep about 'n' slots (0 = unlimited)
        void setMemory(int n) { memory = n; }
        int count() const { return slots; }

        // x: channel-filtered samples, x[0] = absolute index base; phase from slotTiming
        void accumulate(const std::vector<std::complex<float>>& x, uint64_t base, double phase, int variant) {
            const long W = (long)x.size();
            long first = (long)std::ceil(((double)phase - (double)(base % SLOT)));
            while (first < 0) { first += SLOT; }
            for (long ts = first; ts < W; ts += SLOT) {
                long t0 = ts + (long)NTS_FIRST_SYM * SPS;          // centre of training point 0
                long lo = t0 + (long)(K0 - (LMAX - 1)) * SPS + (long)LMIN * SPS - SPS;
                long hi = t0 + (long)(K1 - LMIN) * SPS + (long)(LMAX) * SPS + SPS;
                if (t0 - 4 * SPS < 0 || hi >= W || lo < 0) { continue; }
                // which training sequence: correlation at the nominal timing
                std::complex<float> c[2] = { 0.0f, 0.0f };
                for (int q = 0; q < 2; q++) {
                    for (int k = 0; k < 12; k++) { c[q] += x[t0 + k * SPS] * std::conj(pts[variant][q][k]); }
                }
                int q = std::norm(c[0]) >= std::norm(c[1]) ? 0 : 1;
                double cp = std::norm(c[q]);
                if (avgCorr > 0.0 && cp < 0.25 * avgCorr) { continue; }   // no training sequence here (sync burst etc.)
                avgCorr = avgCorr > 0.0 ? 0.95 * avgCorr + 0.05 * cp : cp;
                double pw = 0.0;
                for (int k = 0; k < 12; k++) { pw += std::norm(x[t0 + k * SPS]); }
                pw = (std::max)(pw / 12.0, 1e-30);
                for (int ph = 0; ph < SPS; ph++) {
                    std::complex<double> r[NE];
                    for (int e = 0; e < NE; e++) { r[e] = std::complex<double>(x[t0 + (K0 + e) * SPS + ph - SPS / 2]); }
                    std::complex<double> h[NT];
                    for (int l = 0; l < NT; l++) {
                        h[l] = 0.0;
                        for (int e = 0; e < NE; e++) { h[l] += pinv[variant][q][l][e] * r[e]; }
                    }
                    // noise from the LS residual; its share in |h|^2 is removed (unbiased profile)
                    double res = 0.0;
                    for (int e = 0; e < NE; e++) {
                        std::complex<double> y(0.0, 0.0);
                        for (int l = 0; l < NT; l++) { y += smat[variant][q][e][l] * h[l]; }
                        res += std::norm(r[e] - y);
                    }
                    double sigma2 = res / (double)(NE - NT);
                    for (int l = 0; l < NT; l++) {
                        double& a = acc[(size_t)(l * SPS + ph)];
                        if (memory > 0) { a *= 1.0 - 1.0 / memory; }
                        a += (std::norm(h[l]) - sigma2 * pnorm[variant][q][l]) / pw;
                    }
                }
                slots++;
            }
        }

        // delay profile normalised to its peak; bin b = delay (b - SPS/2 + LMIN*SPS) samples
        std::vector<float> profile() const {
            std::vector<float> p(NBINS, 0.0f);
            double mx = *std::max_element(acc.begin(), acc.end());
            if (mx <= 0) { return p; }
            for (int i = 0; i < NBINS; i++) { p[i] = (float)((std::max)(0.0, acc[i]) / mx); }
            return p;
        }
        static double binDelaySamples(int b) { return (double)b - SPS / 2 + (double)LMIN * SPS; }

    private:
        void build() {
            acc.assign(NBINS, 0.0);
            for (int v = 0; v < 2; v++) {
                for (int q = 0; q < 2; q++) {
                    pts[v][q] = tsPoints(q == 0 ? NTS1 : NTS2, v == 1);
                    // S[e][l] = s[k - l'] with k = K0+e, l' = LMIN + l
                    std::complex<double> S[NE][NT];
                    for (int e = 0; e < NE; e++) {
                        for (int l = 0; l < NT; l++) { S[e][l] = std::complex<double>(pts[v][q][(K0 + e) - (LMIN + l)]); }
                    }
                    // pinv = (S^H S)^-1 S^H by Gauss-Jordan on the 5x5 normal matrix
                    std::complex<double> A[NT][2 * NT];
                    for (int i = 0; i < NT; i++) {
                        for (int j = 0; j < NT; j++) {
                            std::complex<double> sum(0.0, 0.0);
                            for (int e = 0; e < NE; e++) { sum += std::conj(S[e][i]) * S[e][j]; }
                            A[i][j] = sum;
                            A[i][NT + j] = (i == j) ? 1.0 : 0.0;
                        }
                    }
                    for (int c = 0; c < NT; c++) {
                        int pr = c;
                        for (int r = c + 1; r < NT; r++) { if (std::abs(A[r][c]) > std::abs(A[pr][c])) { pr = r; } }
                        for (int j = 0; j < 2 * NT; j++) { std::swap(A[c][j], A[pr][j]); }
                        std::complex<double> d = A[c][c];
                        for (int j = 0; j < 2 * NT; j++) { A[c][j] /= d; }
                        for (int r = 0; r < NT; r++) {
                            if (r == c) { continue; }
                            std::complex<double> f = A[r][c];
                            for (int j = 0; j < 2 * NT; j++) { A[r][j] -= f * A[c][j]; }
                        }
                    }
                    for (int l = 0; l < NT; l++) {
                        pnorm[v][q][l] = 0.0;
                        for (int e = 0; e < NE; e++) {
                            std::complex<double> sum(0.0, 0.0);
                            for (int j = 0; j < NT; j++) { sum += A[l][NT + j] * std::conj(S[e][j]); }
                            pinv[v][q][l][e] = sum;
                            pnorm[v][q][l] += std::norm(sum);
                        }
                    }
                    for (int e = 0; e < NE; e++) { for (int l = 0; l < NT; l++) { smat[v][q][e][l] = S[e][l]; } }
                }
            }
        }

        std::vector<std::complex<float>> pts[2][2];
        std::complex<double> pinv[2][2][NT][NE];
        std::complex<double> smat[2][2][NE][NT];
        double pnorm[2][2][NT];
        std::vector<double> acc;
        double avgCorr = 0.0;
        int slots = 0;
        int memory = 0;
    };

    struct MultipathResult {
        bool valid = false;
        std::vector<float> measured, ideal;   // delay profiles (ChannelEstimator bins)
        float echoDelayUs = NAN, echoLevelDb = NAN;   // strongest excess component beyond 1 symbol
        float extraSpreadUs = NAN;                     // rms delay spread beyond the ideal one
    };

    // compare a measured delay profile with the ideal one (ChannelEstimator bins)
    inline MultipathResult compareProfiles(const std::vector<float>& meas, const std::vector<float>& ideal) {
        MultipathResult r;
        r.measured = meas;
        r.ideal = ideal;
        if (meas.empty() || meas.size() != ideal.size()) { return r; }
        r.valid = true;
        int pkM = (int)(std::max_element(meas.begin(), meas.end()) - meas.begin());
        int pkI = (int)(std::max_element(ideal.begin(), ideal.end()) - ideal.begin());
        int shift = pkM - pkI;   // align the main paths
        double best = 0.0;
        int bestB = -1;
        const int n = (int)meas.size();
        for (int b = 0; b < n; b++) {
            if (std::abs(b - pkM) <= SPS) { continue; }
            int bi = b - shift;
            double id = (bi >= 0 && bi < n) ? ideal[bi] : 0.0;
            double ex = (double)meas[b] - id;
            if (ex > best) { best = ex; bestB = b; }
        }
        if (bestB >= 0 && best > 0.03) {   // -15 dB
            r.echoDelayUs = (float)((bestB - pkM) / FS * 1e6);
            r.echoLevelDb = (float)(10.0 * std::log10(best));
        }
        auto spread = [&](const std::vector<float>& p) {
            double s0 = 0, s1 = 0, s2 = 0;
            for (int b = 0; b < n; b++) { double w = p[b] > 0.02f ? p[b] : 0.0; s0 += w; s1 += w * b; s2 += w * (double)b * b; }
            if (s0 <= 0) { return 0.0; }
            double m = s1 / s0;
            return std::sqrt((std::max)(0.0, s2 / s0 - m * m));
        };
        double sm = spread(meas), si = spread(ideal);
        r.extraSpreadUs = (float)(std::sqrt((std::max)(0.0, sm * sm - si * si)) / FS * 1e6);
        return r;
    }
}
