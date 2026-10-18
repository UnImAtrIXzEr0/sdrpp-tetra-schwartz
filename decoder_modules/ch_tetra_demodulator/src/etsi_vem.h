#pragma once
// Modulation accuracy of the received TETRA downlink as defined for the
// conformance test (EN 300 394-1 V3.3.1, 7.3.1 and 10.1.3; model of
// EN 300 392-2, 6.6.1.2):
//   Z(k) = [C0 + C1 (S(k) + E(k))] W^k
// with the ideal symbols S(k), the vector error E(k), the residual carrier
// (origin offset) C0, the complex amplitude C1 and W for frequency offset and
// amplitude change. Per burst: RMS vector error over all 255 symbols, peak
// vector error, residual carrier |C0|/|C1| and the frequency error.
//
// Sampling system as in annex B.1.2: TETRA filter (root raised cosine,
// roll-off 0.35, 33 symbols long, B.1.2.2 asks for at least 30), 8 samples
// per symbol (B.1.2.1: at least 4).
//
// The ideal symbols S(k) are exact: a burst is only used when all of its
// blocks decode with a valid CRC (BSCH, SCH/F, SCH/HD, BNCH, STCH) and the
// broadcast block (AACH, Reed-Muller 30,14) decodes; the burst is then coded
// again bit by bit as the base station sent it. Traffic bursts (speech) have
// no such check and are left out.
//
// No SDR++ dependencies: unit-testable. The coding functions are the ones of
// the built-in decoder (lower MAC / PHY, C).

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
    uint32_t tetra_scramb_get_init(uint16_t mcc, uint16_t mnc, uint8_t colour);
    int tetra_scramb_bits(uint32_t lfsr_init, uint8_t* out, int len);
    void block_interleave(uint32_t K, uint32_t a, const uint8_t* in, uint8_t* out);
    void block_deinterleave(uint32_t K, uint32_t a, const uint8_t* in, uint8_t* out);
    struct conv_enc_state;
    int conv_enc_input(struct conv_enc_state* ces, uint8_t* in, int len, uint8_t* out);
    int conv_enc_init(struct conv_enc_state* ces);
    int get_punctured_rate(int pu, uint8_t* in, uint32_t len, uint8_t* out);
    int tetra_rcpc_depunct(int pu, const uint8_t* in, uint32_t len, uint8_t* out);
    void viterbi_dec_sb1_wrapper(const uint8_t* in, uint8_t* out, unsigned int sym_count);
    uint16_t crc16_ccitt_bits(uint8_t* bits, unsigned int len);
    void tetra_rm3014_init(void);
    uint32_t tetra_rm3014_compute(const uint16_t in);
    int build_sync_c_d_burst(uint8_t* buf, const uint8_t* sb, const uint8_t* bb, const uint8_t* bkn);
    int build_norm_c_d_burst(uint8_t* buf, const uint8_t* bkn1, const uint8_t* bb, const uint8_t* bkn2, int two_log_chan);
}

namespace etsi_vem {
    static constexpr double PI_D = 3.14159265358979323846;
    static constexpr double FS = 144000.0;
    static constexpr double SYMRATE = 18000.0;
    static constexpr int SPS = 8;
    static constexpr int NSYM = 255;
    static constexpr int NBITS = 510;
    static constexpr int SLOT = NSYM * SPS;
    static constexpr int FILTER_SYM = 33;          // TETRA filter length (B.1.2.2: >= 30 symbols)
    static constexpr int PUNCT_2_3 = 0;            // TETRA_RCPC_PUNCT_2_3
    static constexpr uint32_t SCRAMB_SB1 = 3;      // SB1 (BSCH) is scrambled with the default code
    static constexpr uint16_t CRC_OK = 0x1d0f;

    // ---------------- TETRA filter ----------------
    // root raised cosine, roll-off 0.35, symbol duration 500/9 us
    inline std::vector<float> tetraFilter(int spanSym = FILTER_SYM, int sps = SPS) {
        const double a = 0.35;
        int n = spanSym * sps + 1;
        std::vector<float> h(n);
        double sum = 0.0;
        for (int i = 0; i < n; i++) {
            double t = (double)(i - n / 2) / sps;
            double v;
            if (std::fabs(t) < 1e-9) { v = 1.0 - a + 4.0 * a / PI_D; }
            else if (std::fabs(std::fabs(t) - 1.0 / (4.0 * a)) < 1e-9) {
                v = a / std::sqrt(2.0) * ((1 + 2 / PI_D) * std::sin(PI_D / (4 * a)) + (1 - 2 / PI_D) * std::cos(PI_D / (4 * a)));
            }
            else { v = (std::sin(PI_D * t * (1 - a)) + 4 * a * t * std::cos(PI_D * t * (1 + a))) / (PI_D * t * (1 - (4 * a * t) * (4 * a * t))); }
            h[i] = (float)v;
            sum += v;
        }
        for (auto& v : h) { v = (float)(v / sum); }
        return h;
    }

    // y[i] = sum h[k] x[i + k - n/2] (centred, no delay); edges are left at 0
    inline std::vector<std::complex<float>> filterCentred(const std::vector<std::complex<float>>& x, const std::vector<float>& h) {
        const int n = (int)h.size(), half = n / 2, W = (int)x.size();
        std::vector<std::complex<float>> y(W, std::complex<float>(0.0f, 0.0f));
        for (int i = half; i + half < W; i++) {
            float re = 0.0f, im = 0.0f;
            const std::complex<float>* p = &x[i - half];
            for (int k = 0; k < n; k++) { re += h[k] * p[k].real(); im += h[k] * p[k].imag(); }
            y[i] = std::complex<float>(re, im);
        }
        return y;
    }

    // ---------------- pi/4-DQPSK ----------------
    // phase change of a dibit in units of pi/4 (EN 300 392-2, 5.5.2):
    // 00 +1, 01 +3, 11 -3, 10 -1
    inline int dibitPhase(int b1, int b2) {
        if (!b1 && !b2) { return 1; }
        if (!b1 && b2) { return 3; }
        if (b1 && b2) { return -3; }
        return -1;
    }
    inline void phaseDibit(int q, uint8_t* b) {   // q in {+1,+3,-3,-1}
        switch (q) {
        case 1: b[0] = 0; b[1] = 0; break;
        case 3: b[0] = 0; b[1] = 1; break;
        case -3: b[0] = 1; b[1] = 1; break;
        default: b[0] = 1; b[1] = 0; break;
        }
    }
    // ideal symbols S(0..254) of a burst, S(-1) = 1; mirror: other rotation sense
    inline std::vector<std::complex<double>> idealSymbols(const uint8_t* bits, bool mirror) {
        std::vector<std::complex<double>> s(NSYM);
        int ph = 0;   // units of pi/4
        for (int k = 0; k < NSYM; k++) {
            int d = dibitPhase(bits[2 * k], bits[2 * k + 1]);
            ph = (ph + (mirror ? -d : d) + 8000) % 8;
            s[k] = std::polar(1.0, ph * PI_D / 4.0);
        }
        return s;
    }

    // ---------------- burst reconstruction ----------------
    enum BurstType { BT_NONE = 0, BT_NDB1, BT_NDB2, BT_SYNC };

    struct Recon {
        bool ok = false;
        BurstType type = BT_NONE;
        uint8_t bits[NBITS] = {};   // the burst as sent
        int bitErrors = 0;          // received vs. reconstructed bits
        int paDeviations = 0;       // phase adjustment pairs sent unlike EN 300 392-2 9.4.4.3.6
        bool sb1 = false;           // BSCH decoded: cell identity below
        uint16_t mcc = 0, mnc = 0;
        uint8_t cc = 0;
        const char* why = "";
    };

    static const uint8_t N_BITS[22] = { 1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,1, 0,0 };
    static const uint8_t P_BITS[22] = { 0,1, 1,1, 1,0, 1,0, 0,1, 0,0, 0,0, 1,1, 0,1, 1,1, 1,0 };
    static const uint8_t Y_BITS[38] = { 1,1, 0,0, 0,0, 0,1, 1,0, 0,1, 1,1, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,0, 0,0, 0,1, 1,0, 0,1, 1,1 };

    inline int hamming(const uint8_t* a, const uint8_t* b, int n) {
        int d = 0;
        for (int i = 0; i < n; i++) { d += (a[i] & 1) != (b[i] & 1); }
        return d;
    }
    inline uint32_t bitsToUint(const uint8_t* b, int n) {
        uint32_t v = 0;
        for (int i = 0; i < n; i++) { v = (v << 1) | (b[i] & 1); }
        return v;
    }

    // one coded block (type-5 bits): de-scramble, de-interleave, de-puncture,
    // Viterbi, CRC; on success code it again into 'out' (type-5 bits)
    inline bool recodeBlock(const uint8_t* rx, int K, int a, int t2, int t1, uint32_t scr, uint8_t* out, uint8_t* type1 = nullptr) {
        uint8_t t4[432], t3[432], t2b[288], mother[288 * 4], t3dp[288 * 4];
        memcpy(t4, rx, K);
        tetra_scramb_bits(scr, t4, K);
        block_deinterleave(K, a, t4, t3);
        memset(t3dp, 0xff, sizeof(t3dp));
        tetra_rcpc_depunct(PUNCT_2_3, t3, K, t3dp);
        viterbi_dec_sb1_wrapper(t3dp, t2b, t2);
        if (crc16_ccitt_bits(t2b, t1 + 16) != CRC_OK) { return false; }
        for (int i = t1 + 16; i < t2; i++) { t2b[i] = 0; }   // tail bits
        if (type1) { memcpy(type1, t2b, t1); }
        struct { uint8_t d[4]; } ces;
        conv_enc_init((struct conv_enc_state*)&ces);
        conv_enc_input((struct conv_enc_state*)&ces, t2b, t2, mother);
        get_punctured_rate(PUNCT_2_3, mother, K, t3);
        block_interleave(K, a, t3, t4);
        tetra_scramb_bits(scr, t4, K);
        memcpy(out, t4, K);
        return true;
    }

    // broadcast block: Reed-Muller (30,14), maximum likelihood over all
    // 16384 code words; accepted with at most 'maxErr' bit errors
    inline bool recodeAach(const uint8_t* rx, uint32_t scr, uint8_t* out, int maxErr = 3) {
        static bool init = false;
        static uint32_t cw[16384];
        if (!init) {
            tetra_rm3014_init();
            for (uint32_t i = 0; i < 16384; i++) { cw[i] = tetra_rm3014_compute((uint16_t)i); }
            init = true;
        }
        uint8_t b[30];
        memcpy(b, rx, 30);
        tetra_scramb_bits(scr, b, 30);
        uint32_t v = bitsToUint(b, 30);
        int best = 99;
        uint32_t bestW = 0;
        for (uint32_t i = 0; i < 16384; i++) {
            uint32_t x = (cw[i] ^ v) & 0x3fffffffu;
            int d = 0;
            while (x) { x &= x - 1; d++; }
            if (d < best) { best = d; bestW = cw[i]; }
        }
        if (best > maxErr) { return false; }
        for (int i = 0; i < 30; i++) { b[i] = (uint8_t)((bestW >> (29 - i)) & 1); }
        tetra_scramb_bits(scr, b, 30);
        memcpy(out, b, 30);
        return true;
    }

    // phase adjustment bits (EN 300 392-2 V3.8.1, 9.4.4.3.6, equation 9.14):
    // their phase change is minus the sum of the phase changes of symbols n1..n2
    inline void phaseAdjust(const uint8_t* bits, int n1, int n2, uint8_t* out) {
        int sum = 0;
        for (int n = n1; n <= n2; n++) { sum += dibitPhase(bits[2 * n - 2], bits[2 * n - 1]); }
        int q = ((-sum) % 8 + 8) % 8;   // 1, 3, 5 or 7 (the sum of an odd count of odd steps is odd)
        phaseDibit(q == 1 ? 1 : q == 3 ? 3 : q == 5 ? -3 : -1, out);
    }
    // set both pairs of a continuous downlink burst per the standard; with
    // 'rx' the pairs actually sent are kept where they differ (counted)
    inline int setPhaseAdjust(uint8_t* bits, bool sync, const uint8_t* rx) {
        uint8_t a[2], b[2];
        // pair 1: symbol 7 (bits 13/14), pair 2: symbol 250 (bits 499/500)
        phaseAdjust(bits, 8, sync ? 108 : 122, a);
        bits[12] = a[0]; bits[13] = a[1];
        phaseAdjust(bits, sync ? 109 : 123, 249, b);
        bits[498] = b[0]; bits[499] = b[1];
        int dev = 0;
        if (rx) {
            if (rx[12] != bits[12] || rx[13] != bits[13]) { dev++; bits[12] = rx[12]; bits[13] = rx[13]; }
            if (rx[498] != bits[498] || rx[499] != bits[499]) { dev++; bits[498] = rx[498]; bits[499] = rx[499]; }
        }
        return dev;
    }

    // rx: 510 demodulated bits of a continuous downlink burst; scr: scrambling
    // code of the cell (0 = not known yet: only synchronisation bursts)
    inline Recon reconstruct(const uint8_t* rx, uint32_t scr) {
        Recon r;
        int dN = hamming(rx + 244, N_BITS, 22), dP = hamming(rx + 244, P_BITS, 22), dY = hamming(rx + 214, Y_BITS, 38);
        int best = (std::min)({ dN, dP, dY });
        if (best > 3) { r.why = "no training sequence"; return r; }
        if (dY == best) {
            r.type = BT_SYNC;
            uint8_t sb[120], bb[30], bk[216], t1[60];
            if (!recodeBlock(rx + 94, 120, 11, 80, 60, SCRAMB_SB1, sb, t1)) { r.why = "BSCH CRC"; return r; }
            r.sb1 = true;
            r.cc = (uint8_t)bitsToUint(t1 + 4, 6);
            r.mcc = (uint16_t)bitsToUint(t1 + 31, 10);
            r.mnc = (uint16_t)bitsToUint(t1 + 41, 14);
            uint32_t cell = tetra_scramb_get_init(r.mcc, r.mnc, r.cc);
            if (scr == 0) { scr = cell; }
            if (!recodeAach(rx + 252, scr, bb)) { r.why = "AACH"; return r; }
            if (!recodeBlock(rx + 282, 216, 101, 144, 124, scr, bk)) { r.why = "SB2 CRC"; return r; }
            build_sync_c_d_burst(r.bits, sb, bb, bk);
        }
        else {
            if (scr == 0) { r.why = "cell not known yet"; return r; }
            uint8_t bb[30], rxbb[30], b1[216], b2[216];
            memcpy(rxbb, rx + 230, 14);
            memcpy(rxbb + 14, rx + 266, 16);
            if (!recodeAach(rxbb, scr, bb)) { r.why = "AACH"; return r; }
            if (dN == best) {
                r.type = BT_NDB1;
                uint8_t blk[432], out[432];
                memcpy(blk, rx + 14, 216);
                memcpy(blk + 216, rx + 282, 216);
                if (!recodeBlock(blk, 432, 103, 288, 268, scr, out)) { r.why = "SCH/F CRC (or traffic)"; return r; }
                memcpy(b1, out, 216);
                memcpy(b2, out + 216, 216);
                build_norm_c_d_burst(r.bits, b1, bb, b2, 0);
            }
            else {
                r.type = BT_NDB2;
                if (!recodeBlock(rx + 14, 216, 101, 144, 124, scr, b1)) { r.why = "block 1 CRC"; return r; }
                if (!recodeBlock(rx + 282, 216, 101, 144, 124, scr, b2)) { r.why = "block 2 CRC"; return r; }
                build_norm_c_d_burst(r.bits, b1, bb, b2, 1);
            }
        }
        // the phase adjustment bits are computed here per the standard (the
        // decoder's burst builder does not follow equation 9.14); pairs the
        // base station sends differently are kept as sent and reported
        r.paDeviations = setPhaseAdjust(r.bits, r.type == BT_SYNC, rx);
        r.bitErrors = hamming(rx, r.bits, NBITS);
        r.ok = true;
        return r;
    }

    // ---------------- sampling and fit ----------------
    // band-limited interpolation (windowed sinc, 16 taps) at fractional position t
    inline std::complex<double> interp(const std::vector<std::complex<float>>& y, double t) {
        int i0 = (int)std::floor(t);
        double f = t - i0;
        std::complex<double> acc(0.0, 0.0);
        const int H = 8;
        if (i0 - H + 1 < 0 || i0 + H >= (int)y.size()) { return acc; }
        if (f < 1e-9) { return std::complex<double>(y[i0]); }
        for (int k = -H + 1; k <= H; k++) {
            double x = k - f;
            double s = std::sin(PI_D * x) / (PI_D * x);
            double w = 0.42 + 0.5 * std::cos(PI_D * x / H) + 0.08 * std::cos(2.0 * PI_D * x / H);   // Blackman
            acc += std::complex<double>(y[i0 + k]) * (s * w);
        }
        return acc;
    }

    // windowed-sinc interpolation table: 1024 fractional positions x 16 taps
    struct InterpTable {
        static constexpr int H = 8, NF = 1024;
        float w[NF + 1][2 * H];
        InterpTable() {
            for (int fi = 0; fi <= NF; fi++) {
                double f = (double)fi / NF;
                for (int j = 0; j < 2 * H; j++) {
                    double x = (j - H + 1) - f;
                    double sc = std::fabs(x) < 1e-12 ? 1.0 : std::sin(PI_D * x) / (PI_D * x);
                    w[fi][j] = (float)(sc * (0.42 + 0.5 * std::cos(PI_D * x / H) + 0.08 * std::cos(2.0 * PI_D * x / H)));
                }
            }
        }
    };
    inline const InterpTable& interpTable() { static const InterpTable t; return t; }

    // samples at the symbol times: position of symbol k = t0 + 127*SPS + (k - 127)*sps
    // (sps != SPS: sample clock of the SDR off against the symbol rate)
    inline void interpRow(const std::vector<std::complex<float>>& y, double t0, int n, std::complex<double>* out, double sps = SPS) {
        const InterpTable& T = interpTable();
        const int H = InterpTable::H;
        const double c = t0 + 127.0 * SPS;
        for (int k = 0; k < n; k++) {
            double t = c + (k - 127) * sps;
            int i0 = (int)std::floor(t);
            int fi = (int)std::lround((t - i0) * InterpTable::NF);
            if (fi == InterpTable::NF) { fi = 0; i0++; }
            if (i0 - H + 1 < 0 || i0 + H >= (int)y.size()) { out[k] = 0.0; continue; }
            const float* w = T.w[fi];
            const std::complex<float>* p = &y[i0 - H + 1];
            double re = 0.0, im = 0.0;
            for (int j = 0; j < 2 * H; j++) { re += w[j] * p[j].real(); im += w[j] * p[j].imag(); }
            out[k] = std::complex<double>(re, im);
        }
    }

    struct Fit {
        bool ok = false;
        double rmsVe = NAN;        // RMS vector error (0.1 = 10 %)
        double peakVe = NAN;       // peak vector error magnitude
        double residualCarrier = NAN;   // |C0| / |C1|
        double freqHz = NAN;       // frequency offset of the burst vs. the tap
        double ampDroopDb = NAN;   // amplitude change over the burst
        double timing = 0.0;       // refined position of symbol 0 (samples)
        double sps = SPS;          // samples per symbol found (sample clock vs. symbol rate)
        double clockPpm = 0.0;     // (sps / 8 - 1) * 1e6
        double cost = INFINITY;
        int peakSym = -1;
    };

    // evaluate the model for one timing / frequency / amplitude slope
    inline Fit evalModel(const std::vector<std::complex<float>>& y, double t0, const std::vector<std::complex<double>>& S, double w, double dr, bool keepE = false, std::vector<double>* emag = nullptr, double sps = SPS) {
        Fit f;
        const int N = (int)S.size();
        std::vector<std::complex<double>> Y(N);
        interpRow(y, t0, N, Y.data(), sps);
        {
            std::complex<double> step = std::exp(std::complex<double>(-dr, -w)), rot(1.0, 0.0);
            for (int k = 0; k < N; k++) { Y[k] *= rot; rot *= step; }
        }
        // least squares C0, C1: Y = C0 + C1 S
        std::complex<double> sS(0, 0), sY(0, 0), sYS(0, 0);
        double sSS = 0.0;
        for (int k = 0; k < N; k++) { sS += S[k]; sY += Y[k]; sYS += Y[k] * std::conj(S[k]); sSS += std::norm(S[k]); }
        double n = N;
        std::complex<double> det = n * sSS - std::norm(sS);
        if (std::abs(det) < 1e-12) { return f; }
        std::complex<double> C1 = (n * sYS - sY * std::conj(sS)) / det;
        std::complex<double> C0 = (sY - C1 * sS) / n;
        if (std::abs(C1) < 1e-12) { return f; }
        double sum = 0.0, pk = 0.0;
        int pki = -1;
        if (emag) { emag->assign(N, 0.0); }
        for (int k = 0; k < N; k++) {
            std::complex<double> E = (Y[k] - C0) / C1 - S[k];
            double m2 = std::norm(E);
            sum += m2;
            if (m2 > pk) { pk = m2; pki = k; }
            if (emag) { (*emag)[k] = std::sqrt(m2); }
        }
        (void)keepE;
        f.ok = true;
        f.cost = sum;
        f.rmsVe = std::sqrt(sum / N);
        f.peakVe = std::sqrt(pk);
        f.peakSym = pki;
        f.residualCarrier = std::abs(C0) / std::abs(C1);
        f.freqHz = w / (2.0 * PI_D) * SYMRATE;
        f.ampDroopDb = 20.0 * std::log10(std::exp(dr * (N - 1)));
        f.timing = t0;
        f.sps = sps;
        f.clockPpm = (sps / SPS - 1.0) * 1e6;
        return f;
    }

    // initial frequency and amplitude slope from D(k) = Z(k) conj(S(k))
    inline void slopes(const std::vector<std::complex<float>>& y, double t0, const std::vector<std::complex<double>>& S, double& w, double& dr, double sps = SPS) {
        const int N = (int)S.size();
        std::vector<double> ph(N), lm(N);
        std::vector<std::complex<double>> Z(N);
        interpRow(y, t0, N, Z.data(), sps);
        double prev = 0.0;
        for (int k = 0; k < N; k++) {
            std::complex<double> d = Z[k] * std::conj(S[k]);
            double a = std::arg(d);
            if (k > 0) { while (a - prev > PI_D) { a -= 2 * PI_D; } while (a - prev < -PI_D) { a += 2 * PI_D; } }
            ph[k] = a;
            prev = a;
            lm[k] = std::log((std::max)(std::abs(d), 1e-30));
        }
        double mk = (N - 1) / 2.0, sxx = 0.0, sxp = 0.0, sxl = 0.0, mp = 0.0, ml = 0.0;
        for (int k = 0; k < N; k++) { mp += ph[k]; ml += lm[k]; }
        mp /= N; ml /= N;
        for (int k = 0; k < N; k++) { double x = k - mk; sxx += x * x; sxp += x * (ph[k] - mp); sxl += x * (lm[k] - ml); }
        w = sxp / sxx;
        dr = sxl / sxx;
    }

    // golden section search of f on [a, b]
    template <class F>
    inline double golden(F f, double a, double b, int it) {
        const double g = 0.6180339887498949;
        double c = b - g * (b - a), d = a + g * (b - a);
        double fc = f(c), fd = f(d);
        for (int i = 0; i < it; i++) {
            if (fc < fd) { b = d; d = c; fd = fc; c = b - g * (b - a); fc = f(c); }
            else { a = c; c = d; fc = fd; d = a + g * (b - a); fd = f(d); }
        }
        return (a + b) / 2.0;
    }

    // full fit: timing (+-0.6 samples around t0), sample clock (+-300 ppm:
    // the test system of the standard is locked, an SDR is not), frequency,
    // amplitude slope, C0, C1 by least squares of the vector error
    inline Fit fitBurst(const std::vector<std::complex<float>>& y, double t0, const std::vector<std::complex<double>>& S) {
        double w = 0.0, dr = 0.0, sps = SPS;
        auto costTS = [&](double t, double q) {
            double ww, dd;
            slopes(y, t, S, ww, dd, q);
            return evalModel(y, t, S, ww, dd, false, nullptr, q).cost;
        };
        double t = golden([&](double tt) { return costTS(tt, sps); }, t0 - 0.6, t0 + 0.6, 16);
        sps = golden([&](double q) { return costTS(t, q); }, SPS * (1.0 - 3e-4), SPS * (1.0 + 3e-4), 16);
        t = golden([&](double tt) { return costTS(tt, sps); }, t - 0.3, t + 0.3, 12);
        slopes(y, t, S, w, dr, sps);
        // refine the frequency (slope estimate is slightly biased by C0)
        double span = 2.0 * PI_D * 20.0 / SYMRATE;
        w = golden([&](double ww) { return evalModel(y, t, S, ww, dr, false, nullptr, sps).cost; }, w - span, w + span, 20);
        dr = golden([&](double dd) { return evalModel(y, t, S, w, dd, false, nullptr, sps).cost; }, dr - 2e-4, dr + 2e-4, 16);
        t = golden([&](double tt) { return evalModel(y, tt, S, w, dr, false, nullptr, sps).cost; }, t - 0.15, t + 0.15, 14);
        return evalModel(y, t, S, w, dr, false, nullptr, sps);
    }

    // demodulate the 510 bits of a burst (decision on the phase changes);
    // t0 = position of symbol 0 (its phase change is relative to symbol -1)
    inline void demodBits(const std::vector<std::complex<float>>& y, double t0, bool mirror, uint8_t* bits) {
        std::vector<double> d(NSYM);
        std::complex<double> prev = interp(y, t0 - SPS);
        for (int k = 0; k < NSYM; k++) {
            std::complex<double> z = interp(y, t0 + (double)k * SPS);
            d[k] = std::arg(z * std::conj(prev));
            prev = z;
        }
        // frequency offset: mean deviation from the nearest decision, two passes
        double off = 0.0;
        for (int pass = 0; pass < 2; pass++) {
            double s = 0.0;
            for (int k = 0; k < NSYM; k++) {
                double v = d[k] - off;
                double q = std::round((v / (PI_D / 4.0) - 1.0) / 2.0) * 2.0 + 1.0;   // odd multiples of pi/4
                double e = v - q * PI_D / 4.0;
                s += e;
            }
            off += s / NSYM;
        }
        for (int k = 0; k < NSYM; k++) {
            double v = d[k] - off;
            while (v > PI_D) { v -= 2 * PI_D; }
            while (v < -PI_D) { v += 2 * PI_D; }
            int q = (int)(std::round((v / (PI_D / 4.0) - 1.0) / 2.0) * 2.0 + 1.0);
            if (q > 3) { q = -3; }
            if (q < -3) { q = 3; }
            if (mirror) { q = -q; }
            phaseDibit(q, bits + 2 * k);
        }
    }

    // decision-directed error of the phase changes (coarse timing)
    inline double ddError(const std::vector<std::complex<float>>& y, double t0) {
        std::complex<double> prev = interp(y, t0 - SPS);
        std::vector<double> d(NSYM);
        for (int k = 0; k < NSYM; k++) {
            std::complex<double> z = interp(y, t0 + (double)k * SPS);
            d[k] = std::arg(z * std::conj(prev));
            prev = z;
        }
        double mean = 0.0;
        for (int k = 0; k < NSYM; k++) {
            double q = std::round((d[k] / (PI_D / 4.0) - 1.0) / 2.0) * 2.0 + 1.0;
            mean += d[k] - q * PI_D / 4.0;
        }
        mean /= NSYM;
        double s = 0.0;
        for (int k = 0; k < NSYM; k++) {
            double v = d[k] - mean;
            double q = std::round((v / (PI_D / 4.0) - 1.0) / 2.0) * 2.0 + 1.0;
            double e = v - q * PI_D / 4.0;
            s += e * e;
        }
        return s;
    }

    // slot grid of a continuous downlink from the training sequences (normal
    // 1/2 and synchronisation) in the demodulated bits: position of symbol 0
    // of a slot (mod SLOT samples) and the rotation sense. y: TETRA-filtered.
    struct Grid { bool valid = false; double t0 = 0.0; bool mirror = false; int hits = 0; int slots = 0; };
    inline Grid findSlotGrid(const std::vector<std::complex<float>>& y, size_t from, size_t to) {
        Grid g;
        if (to <= from + 3 * SLOT || to > y.size()) { return g; }
        const int nsym = (int)((to - from) / SPS) - 2;
        int bestHits = 0, bestPh = 0, bestPos = 0;
        bool bestMir = false;
        std::vector<uint8_t> bits((size_t)nsym * 2);
        for (int ph = 0; ph < SPS; ph++) {
            // phase changes at this sample phase, frequency offset removed
            std::vector<double> d(nsym);
            std::complex<float> prev = y[from + ph];
            double mean = 0.0;
            for (int k = 0; k < nsym; k++) {
                std::complex<float> z = y[from + ph + (size_t)(k + 1) * SPS];
                d[k] = std::arg(std::complex<double>(z * std::conj(prev)));
                prev = z;
                double q = std::round((d[k] / (PI_D / 4.0) - 1.0) / 2.0) * 2.0 + 1.0;
                mean += d[k] - q * PI_D / 4.0;
            }
            mean /= nsym;
            for (int mir = 0; mir < 2; mir++) {
                for (int k = 0; k < nsym; k++) {
                    double v = d[k] - mean;
                    while (v > PI_D) { v -= 2 * PI_D; }
                    while (v < -PI_D) { v += 2 * PI_D; }
                    int q = (int)(std::round((v / (PI_D / 4.0) - 1.0) / 2.0) * 2.0 + 1.0);
                    if (q > 3) { q = -3; }
                    if (q < -3) { q = 3; }
                    phaseDibit(mir ? -q : q, &bits[2 * (size_t)k]);
                }
                // histogram of burst starts (symbol index mod 255)
                std::vector<int> hist(NSYM, 0);
                for (int k = 0; k + 19 < nsym; k++) {
                    const uint8_t* b = &bits[2 * (size_t)k];
                    if (k + 11 < nsym && (hamming(b, N_BITS, 22) <= 2 || hamming(b, P_BITS, 22) <= 2)) { hist[((k - 122) % NSYM + NSYM) % NSYM]++; }
                    if (hamming(b, Y_BITS, 38) <= 3) { hist[((k - 107) % NSYM + NSYM) % NSYM]++; }
                }
                int pos = (int)(std::max_element(hist.begin(), hist.end()) - hist.begin());
                if (hist[pos] > bestHits) { bestHits = hist[pos]; bestPh = ph; bestPos = pos; bestMir = mir == 1; }
            }
        }
        g.slots = nsym / NSYM;
        g.hits = bestHits;
        // dibit k is the phase change into the symbol at sample from+ph+(k+1)*SPS
        g.t0 = std::fmod((double)from + bestPh + (double)(bestPos + 1) * SPS, (double)SLOT);
        g.mirror = bestMir;
        g.valid = bestHits >= (std::max)(2, g.slots / 3);
        return g;
    }

    struct BurstMeas {
        bool ok = false;
        BurstType type = BT_NONE;
        Fit fit;
        int bitErrors = 0;
        int paDeviations = 0;
        const char* why = "";
        bool sb1 = false;
        uint16_t mcc = 0, mnc = 0;
        uint8_t cc = 0;
    };

    // one burst: y = TETRA-filtered samples (8 per symbol), t0 = expected
    // position of symbol 0 (+-4 samples), scr = cell scrambling code (0: learn from BSCH)
    inline BurstMeas measureBurst(const std::vector<std::complex<float>>& y, double t0, bool mirror, uint32_t scr) {
        BurstMeas m;
        // coarse timing on the decision-directed error
        double best = INFINITY, bt = t0;
        for (int dt = -4; dt <= 4; dt++) {
            double e = ddError(y, t0 + dt);
            if (e < best) { best = e; bt = t0 + dt; }
        }
        uint8_t bits[NBITS];
        demodBits(y, bt, mirror, bits);
        Recon r = reconstruct(bits, scr);
        m.sb1 = r.sb1; m.mcc = r.mcc; m.mnc = r.mnc; m.cc = r.cc;
        m.type = r.type;
        if (!r.ok) { m.why = r.why; return m; }
        m.bitErrors = r.bitErrors;
        m.paDeviations = r.paDeviations;
        std::vector<std::complex<double>> S = idealSymbols(r.bits, mirror);
        m.fit = fitBurst(y, bt, S);
        m.ok = m.fit.ok;
        return m;
    }

    // ---------------- statistics over the last 200 bursts ----------------
    struct Stats {
        bool gridValid = false;
        int n = 0;
        static constexpr int TARGET = 200;   // EN 300 394-1 10.1.3 b)
        uint64_t measured = 0, rejected = 0;
        uint64_t rejTraffic = 0, rejCrc = 0, rejOther = 0;
        double rmsMean = NAN, rmsMax = NAN, peakMax = NAN;
        double c0Mean = NAN, c0Max = NAN;
        double freqMean = NAN, freqStd = NAN;
        double clockPpm = NAN;   // sample clock vs. symbol rate (mean)
        int paDevBursts = 0;
        bool cellKnown = false;
        uint16_t mcc = 0, mnc = 0;
        uint8_t cc = 0;
    };

    // processes overlapping windows of the tap (raw 144 kS/s samples, x[0]
    // at absolute index 'base'); every slot is measured once
    class Engine {
    public:
        Engine() : h(tetraFilter()) {}
        size_t margin() const { return h.size() + 64; }
        size_t window(double seconds) const { return (size_t)(FS * seconds) + 2 * SLOT + 2 * margin(); }
        void reset() { *this = Engine(); }
        void setCellScrambling(uint32_t s) { scrGiven = s; }
        const Stats& stats() const { return st; }

        void process(const std::vector<std::complex<float>>& x, uint64_t base) {
            const size_t M = margin(), W = x.size();
            if (W < 3 * SLOT + 2 * M) { return; }
            uint32_t scr = scrGiven ? scrGiven : scrLearned;
            std::vector<std::complex<float>> y = filterCentred(x, h);
            Grid g = findSlotGrid(y, M, W - M);
            st.gridValid = g.valid;
            if (g.valid) {
                double gridT0 = std::fmod((double)base + g.t0, (double)SLOT);
                double rel = std::fmod(gridT0 - (double)(base % SLOT) + 2.0 * SLOT, (double)SLOT);
                for (double t = rel; t + SLOT + M < (double)W; t += SLOT) {
                    if (t < (double)M) { continue; }
                    uint64_t abs = base + (uint64_t)t;
                    if (doneValid && abs <= doneAbs + SLOT / 2) { continue; }
                    doneAbs = abs;
                    doneValid = true;
                    BurstMeas m = measureBurst(y, t, g.mirror, scr);
                    if (m.sb1) {
                        st.cellKnown = true;
                        st.mcc = m.mcc; st.mnc = m.mnc; st.cc = m.cc;
                        scrLearned = tetra_scramb_get_init(m.mcc, m.mnc, m.cc);
                        if (!scr) { scr = scrLearned; }
                    }
                    if (!m.ok) {
                        st.rejected++;
                        std::string w = m.why;
                        if (w.find("SCH/F") != std::string::npos) { st.rejTraffic++; }
                        else if (w.find("CRC") != std::string::npos || w == "AACH") { st.rejCrc++; }
                        else { st.rejOther++; }
                        continue;
                    }
                    st.measured++;
                    hist.push_back({ m.fit.rmsVe, m.fit.peakVe, m.fit.residualCarrier, m.fit.freqHz, m.paDeviations > 0, m.fit.clockPpm });
                    if ((int)hist.size() > Stats::TARGET) { hist.erase(hist.begin()); }
                }
            }
            if (scrGiven) { st.cellKnown = true; }
            summarise();
        }

    private:
        struct One { double rms, peak, c0, f; bool paDev; double ppm; };
        void summarise() {
            st.n = (int)hist.size();
            if (hist.empty()) { return; }
            double s2 = 0, rmax = 0, pmax = 0, c0s = 0, c0max = 0, fs = 0, fs2 = 0, pp = 0;
            int pd = 0;
            for (auto& o : hist) {
                pp += o.ppm;
                s2 += o.rms * o.rms; rmax = (std::max)(rmax, o.rms); pmax = (std::max)(pmax, o.peak);
                c0s += o.c0; c0max = (std::max)(c0max, o.c0); fs += o.f; fs2 += o.f * o.f;
                pd += o.paDev ? 1 : 0;
            }
            double n = (double)hist.size();
            st.rmsMean = std::sqrt(s2 / n);
            st.rmsMax = rmax;
            st.peakMax = pmax;
            st.c0Mean = c0s / n;
            st.c0Max = c0max;
            st.freqMean = fs / n;
            st.freqStd = std::sqrt((std::max)(0.0, fs2 / n - st.freqMean * st.freqMean));
            st.paDevBursts = pd;
            st.clockPpm = pp / n;
        }
        std::vector<float> h;
        std::vector<One> hist;
        Stats st;
        uint64_t doneAbs = 0;
        bool doneValid = false;
        uint32_t scrGiven = 0, scrLearned = 0;
    };
}
