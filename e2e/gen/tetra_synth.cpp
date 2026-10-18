// Synthetic TETRA downlink recording for the tests and the documentation
// screenshots: a fictitious amateur radio cell, no real network data.
//
//   cell:        MCC 901, MNC 9999, colour code 1, location area 1
//   downlink:    438.2375 MHz (band 4 = 400 MHz, carrier 1529, offset +12.5 kHz)
//   uplink:      7.6 MHz below (duplex spacing field 7, normal operation)
//   stations:    ISSI 10000001 ... 10000006 (8 digits: no DMR ID can match)
//   groups:      GSSI 9 and 91
//
// Bursts are coded with the decoder's own channel coding functions (CRC 16,
// rate 2/3 RCPC, block interleaving, Reed-Muller (30,14), scrambling) and the
// phase adjustment bits per EN 300 392-2 9.4.4.3.6. Modulation: pi/4-DQPSK,
// root raised cosine 0.35, then a weak echo, a small frequency error and noise.
//
// Output: 2-channel 16-bit WAV, 32 kS/s, carrier at -3 kHz (the tests name it
// baseband_438240500Hz_...wav, so the carrier is at 438.2375 MHz).
//
// Build (from the repository root):
//   D=decoder_modules/ch_tetra_demodulator/src
//   gcc -c -O2 -I$D/decoder/src $D/decoder/src/lower_mac/{tetra_scramb,tetra_interleave,tetra_conv_enc,crc_simple,tetra_rm3014}.c $D/decoder/src/phy/tetra_burst.c
//   g++ -O2 -std=c++17 -I$D -I$D/decoder/src e2e/gen/tetra_synth.cpp *.o -o tetra_synth
//   ./tetra_synth e2e/recordings/tetra_synthetic.wav 19

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>
#include "etsi_vem.h"

using cf = std::complex<float>;
using namespace etsi_vem;

static const uint16_t MCC = 901, MNC = 9999;
static const uint8_t CC = 1;
static const uint32_t ISSI[6] = { 10000001, 10000002, 10000003, 10000004, 10000005, 10000006 };

// ---------------- bit helpers ----------------
struct Bits {
    std::vector<uint8_t> b;
    void put(uint32_t v, int n) { for (int i = n - 1; i >= 0; i--) { b.push_back((v >> i) & 1); } }
    void putText(const char* s) { for (; *s; s++) { put((uint8_t)*s, 8); } }
    int size() const { return (int)b.size(); }
};

// type-1 bits -> coded, interleaved, scrambled type-5 bits
static void codeBlock(const uint8_t* t1, int n1, int K, int a, uint32_t scr, uint8_t* out) {
    uint8_t t2[288 + 4] = {}, mother[(288 + 4) * 4], t3[432];
    memcpy(t2, t1, n1);
    uint16_t crc = (uint16_t)~crc16_ccitt_bits(t2, n1);
    for (int i = 0; i < 16; i++) { t2[n1 + i] = (crc >> (15 - i)) & 1; }
    int n2 = n1 + 16 + 4;   // + tail bits (0)
    struct conv_enc_state { uint8_t d[4]; } ces;
    conv_enc_init((struct ::conv_enc_state*)&ces);
    conv_enc_input((struct ::conv_enc_state*)&ces, t2, n2, mother);
    get_punctured_rate(PUNCT_2_3, mother, K, t3);
    block_interleave(K, a, t3, out);
    tetra_scramb_bits(scr, out, K);
}

static void codeAach(uint16_t v, uint32_t scr, uint8_t* out) {
    uint32_t cw = tetra_rm3014_compute(v);
    for (int i = 0; i < 30; i++) { out[i] = (cw >> (29 - i)) & 1; }
    tetra_scramb_bits(scr, out, 30);
}

// ---------------- PDUs ----------------
static Bits syncPdu(int tn, int fn, int mn) {
    Bits p;
    p.put(0x3, 4);              // system code
    p.put(CC, 6);
    p.put(tn - 1, 2);
    p.put(fn, 5);
    p.put(mn, 6);
    p.put(0, 2);                // sharing mode: continuous transmission
    p.put(0, 3);                // TS reserved frames
    p.put(0, 1);                // U-plane DTX
    p.put(0, 1);                // frame 18 extension
    p.put(0, 1);                // reserved
    p.put(MCC, 10);
    p.put(MNC, 14);
    p.put(1, 2);                // neighbour cell broadcast: supported
    p.put(1, 2);                // cell service level / load: low
    p.put(0, 1);                // late entry
    return p;                   // 60 bits
}

static Bits sysinfoPdu(int hyperframe) {
    Bits p;
    p.put(2, 2);                // MAC PDU type: broadcast
    p.put(0, 2);                // SYSINFO
    p.put(1529, 12);            // main carrier
    p.put(4, 4);                // frequency band 400 MHz
    p.put(3, 2);                // offset +12.5 kHz
    p.put(7, 3);                // duplex spacing field (amateur table: 7.6 MHz)
    p.put(0, 1);                // normal operation (uplink below)
    p.put(0, 2);                // number of secondary control channels
    p.put(4, 3);                // MS_TXPWR_MAX_CELL
    p.put(2, 4);                // RXLEV_ACCESS_MIN
    p.put(7, 4);                // ACCESS_PARAMETER
    p.put(9, 4);                // RADIO_DOWNLINK_TIMEOUT
    p.put(0, 1);                // hyperframe number follows
    p.put(hyperframe & 0xffff, 16);
    p.put(2, 2);                // optional field: default definition for access code A
    p.put(0x0f, 4); p.put(0x0f, 4); p.put(0, 3); p.put(0, 3); p.put(0, 1); p.put(0, 1); p.put(0, 4);   // 20 bits
    // MLE part (D-MLE-SYSINFO, 42 bits)
    p.put(1, 14);               // location area
    p.put(0xffff, 16);          // subscriber class
    p.put(0x0c0, 12);           // BS service details: registration, de-registration ...
    return p;                   // 124 bits
}

// MAC-RESOURCE carrying an LLC BL-UDATA with 'sdu' (MLE PD + PDU); fill bits to the octet
static Bits macResource(uint32_t ssi, const Bits& sdu) {
    Bits h;
    int hdr = 2 + 1 + 1 + 2 + 1 + 6 + 3 + 24 + 1 + 1 + 1;
    int total = hdr + 4 + sdu.size();
    int octets = (total + 7) / 8;
    bool fill = total % 8 != 0;
    h.put(0, 2);                // MAC-RESOURCE
    h.put(fill ? 1 : 0, 1);
    h.put(0, 1);                // position of grant
    h.put(0, 2);                // not encrypted
    h.put(0, 1);                // random access flag
    h.put(octets, 6);           // length indication
    h.put(1, 3);                // address type: SSI
    h.put(ssi, 24);
    h.put(0, 1); h.put(0, 1); h.put(0, 1);   // power control, slot granting, channel allocation
    h.put(2, 4);                // LLC BL-UDATA
    h.b.insert(h.b.end(), sdu.b.begin(), sdu.b.end());
    if (fill) { h.put(1, 1); while (h.size() % 8) { h.put(0, 1); } }
    return h;
}

static Bits nullPdu() {
    Bits h;
    h.put(0, 2); h.put(0, 1); h.put(0, 1); h.put(0, 2); h.put(0, 1);
    h.put(2, 6);                // 2 octets
    h.put(0, 3);                // null address
    h.put(0, 16 - 16);
    while (h.size() < 16) { h.put(0, 1); }
    return h;
}

static Bits cmce() { Bits b; b.put(2, 3); return b; }   // MLE PD: CMCE
static Bits mm() { Bits b; b.put(1, 3); return b; }     // MLE PD: MM

static Bits dSetup(int callId, uint32_t gssiCaller) {
    Bits p = cmce();
    p.put(7, 5); p.put(callId, 14);
    p.put(3, 4); p.put(0, 1); p.put(0, 1);  // call timeout, hook method, simplex
    p.put(0x04, 8);                         // basic service: speech, clear, point-to-multipoint
    p.put(0, 2); p.put(0, 1); p.put(0, 4);  // tx grant: granted, tx request permission, priority
    p.put(1, 1);                            // O-bit
    p.put(0, 1); p.put(0, 1);               // no notification, no temporary address
    p.put(1, 1); p.put(1, 2); p.put(gssiCaller, 24);   // calling party SSI
    p.put(0, 1);                            // M-bit: no type 3 elements
    return p;
}
static Bits dTxGranted(int callId, uint32_t talker) {
    Bits p = cmce();
    p.put(11, 5); p.put(callId, 14);
    p.put(0, 2); p.put(0, 1); p.put(0, 1); p.put(0, 1);
    p.put(1, 1); p.put(0, 1);
    p.put(1, 1); p.put(1, 2); p.put(talker, 24);
    p.put(0, 1);
    return p;
}
static Bits dTxCeased(int callId) { Bits p = cmce(); p.put(9, 5); p.put(callId, 14); p.put(0, 1); p.put(0, 1); return p; }
static Bits dRelease(int callId) { Bits p = cmce(); p.put(6, 5); p.put(callId, 14); p.put(1, 5); p.put(0, 1); return p; }
static Bits dSdsText(uint32_t from, const char* text) {
    Bits p = cmce();
    p.put(15, 5);
    p.put(1, 2); p.put(from, 24);           // calling party SSI
    p.put(3, 2);                            // SDS type 4
    Bits u;
    u.put(0x02, 8);                         // simple text messaging
    u.put(0, 1); u.put(1, 7);               // text coding: ISO 8859-1
    u.putText(text);
    p.put(u.size(), 11);
    p.b.insert(p.b.end(), u.b.begin(), u.b.end());
    p.put(0, 1);
    return p;
}
static Bits dStatus(uint32_t from, int status) {
    Bits p = cmce();
    p.put(8, 5); p.put(1, 2); p.put(from, 24); p.put(status, 16); p.put(0, 1);
    return p;
}
static Bits dAttachGroups(const std::vector<uint32_t>& g) {
    Bits p = mm();
    p.put(10, 4);                           // D-ATTACH/DETACH GROUP IDENTITY
    p.put(0, 1); p.put(0, 1); p.put(1, 1);  // report, ack request, mode: new list
    p.put(1, 1);                            // O-bit
    Bits e;
    e.put((uint32_t)g.size(), 6);
    for (uint32_t x : g) { e.put(0, 1); e.put(1, 2); e.put(0, 3); e.put(0, 2); e.put(x, 24); }
    p.put(1, 1); p.put(7, 4); p.put(e.size(), 11);   // type 4 element: group identity downlink
    p.b.insert(p.b.end(), e.b.begin(), e.b.end());
    p.put(0, 1);                            // no more elements
    return p;
}

// ---------------- traffic plan (per multiframe on TS1) ----------------
// a few calls on GSSI 91 / 9 by changing talkers, SDS and status messages
static std::vector<Bits> planned(int mfCount) {
    static const char* texts[3] = { "Test SDS", "QRV on 9", "73 de synth" };
    std::vector<Bits> q;
    for (int m = 0; m < mfCount; m++) {
        int c = m / 6, k = m % 6, call = 101 + c;
        uint32_t talker = ISSI[c % 6], other = ISSI[(c + 3) % 6];
        uint32_t group = (c % 2) ? 9 : 91;
        if (k == 0) { q.push_back(macResource(group, dSetup(call, talker))); q.push_back(macResource(group, dTxGranted(call, talker))); }
        else if (k == 1) { q.push_back(macResource(group, dTxCeased(call))); q.push_back(macResource(group, dTxGranted(call, other))); }
        else if (k == 2) { q.push_back(macResource(group, dTxCeased(call))); q.push_back(macResource(group, dRelease(call))); }
        else if (k == 3) { q.push_back(macResource(ISSI[(c + 1) % 6], dSdsText(ISSI[(c + 4) % 6], texts[c % 3]))); }
        else if (k == 4) { q.push_back(macResource(ISSI[(c + 2) % 6], dStatus(ISSI[(c + 5) % 6], 32768 + c))); }
        else { q.push_back(macResource(ISSI[(c + 1) % 6], dAttachGroups({ 9, 91 }))); }
    }
    return q;
}

// ---------------- WAV ----------------
static void writeWav(const char* path, const std::vector<cf>& x, int fs) {
    FILE* f = fopen(path, "wb");
    if (!f) { perror(path); exit(1); }
    uint32_t data = (uint32_t)x.size() * 4, riff = 36 + data, byteRate = fs * 4;
    uint16_t pcm = 1, ch = 2, align = 4, bps = 16;
    uint32_t fmtLen = 16, ufs = fs;
    fwrite("RIFF", 1, 4, f); fwrite(&riff, 4, 1, f); fwrite("WAVEfmt ", 1, 8, f);
    fwrite(&fmtLen, 4, 1, f); fwrite(&pcm, 2, 1, f); fwrite(&ch, 2, 1, f); fwrite(&ufs, 4, 1, f);
    fwrite(&byteRate, 4, 1, f); fwrite(&align, 2, 1, f); fwrite(&bps, 2, 1, f);
    fwrite("data", 1, 4, f); fwrite(&data, 4, 1, f);
    for (auto& v : x) {
        int16_t s[2] = { (int16_t)std::lround(std::clamp(v.real(), -1.0f, 1.0f) * 32767.0f),
                         (int16_t)std::lround(std::clamp(v.imag(), -1.0f, 1.0f) * 32767.0f) };
        fwrite(s, 2, 2, f);
    }
    fclose(f);
}

int main(int argc, char** argv) {
    if (argc < 2) { fprintf(stderr, "usage: %s out.wav [seconds] [bits.out]\n", argv[0]); return 1; }
    double seconds = argc > 2 ? atof(argv[2]) : 19.0;
    tetra_rm3014_init();
    const uint32_t scr = tetra_scramb_get_init(MCC, MNC, CC);
    const int nSlots = (int)(seconds * SYMRATE / NSYM) + 8;
    const int nMf = nSlots / (4 * 18) + 2;
    std::vector<Bits> queue = planned(nMf);
    size_t qi = 0;
    std::vector<uint8_t> allBits;
    allBits.reserve((size_t)nSlots * NBITS);
    int hyper = 7, mn = 1, fn = 1, tn = 1;
    for (int s = 0; s < nSlots; s++) {
        uint8_t burst[NBITS];
        uint8_t bb[30];
        bool sync = (fn == 18);
        if (sync) {
            Bits sp = syncPdu(tn, fn, mn), si = sysinfoPdu(hyper);
            uint8_t sb[120], bk[216];
            codeBlock(sp.b.data(), 60, 120, 11, SCRAMB_SB1, sb);
            codeAach(0, scr, bb);           // ACCESS-ASSIGN: header 00, both fields 0
            codeBlock(si.b.data(), 124, 216, 101, scr, bk);
            build_sync_c_d_burst(burst, sb, bb, bk);
        }
        else {
            // SCH/F: on TS1 the planned PDUs (frames 1 and 9), else Null PDU
            Bits blk;
            if (tn == 1 && (fn == 1 || fn == 9) && qi < queue.size()) { blk = queue[qi++]; }
            Bits np = nullPdu();
            blk.b.insert(blk.b.end(), np.b.begin(), np.b.end());
            blk.b.resize(268, 0);
            uint8_t out[432];
            codeBlock(blk.b.data(), 268, 432, 103, scr, out);
            codeAach((uint16_t)(0x00 << 12 | 0x05 << 6 | 0x05), scr, bb);   // header 00, access fields
            build_norm_c_d_burst(burst, out, bb, out + 216, 0);
        }
        setPhaseAdjust(burst, sync, nullptr);
        // check: the burst must decode again
        Recon r = reconstruct(burst, scr);
        if (!r.ok || r.bitErrors) { fprintf(stderr, "slot %d (fn %d tn %d): %s, %d bit errors\n", s, fn, tn, r.why, r.bitErrors); return 2; }
        allBits.insert(allBits.end(), burst, burst + NBITS);
        if (++tn > 4) { tn = 1; if (++fn > 18) { fn = 1; if (++mn > 60) { mn = 1; hyper++; } } }
    }
    fprintf(stderr, "%d slots, %zu of %zu planned PDUs sent\n", nSlots, qi, queue.size());
    if (argc > 3) { FILE* f = fopen(argv[3], "wb"); fwrite(allBits.data(), 1, allBits.size(), f); fclose(f); }

    // ---- pi/4-DQPSK at 144 kS/s (8 samples per symbol), TETRA filter ----
    const int nSym = (int)allBits.size() / 2;
    std::vector<cf> imp((size_t)nSym * SPS, cf(0.0f, 0.0f));
    int ph = 0;
    for (int k = 0; k < nSym; k++) {
        ph = (ph + dibitPhase(allBits[2 * k], allBits[2 * k + 1]) + 8) % 8;
        imp[(size_t)k * SPS] = std::polar(1.0f, (float)(ph * PI_D / 4.0)) * (float)SPS;
    }
    std::vector<cf> y = filterCentred(imp, tetraFilter(33, SPS));
    // weak echo: 20.8 us (3 samples), -18 dB
    {
        cf g = std::polar(0.126f, 0.7f);
        for (size_t i = y.size() - 1; i >= 3; i--) { y[i] += g * y[i - 3]; }
    }
    // ---- 144 kS/s -> 32 kS/s (x2, low-pass, /9) ----
    const int L = 2, M = 9, NT = 511;
    std::vector<float> h(NT);
    {
        double fc = 14500.0 / (144000.0 * L), sum = 0.0;
        auto i0 = [](double x) { double s = 1, t = 1; for (int k = 1; k < 60; k++) { t *= (x / (2 * k)) * (x / (2 * k)); s += t; } return s; };
        for (int n = 0; n < NT; n++) {
            int k = n - NT / 2;
            double si = k == 0 ? 2 * fc : std::sin(2 * PI_D * fc * k) / (PI_D * k);
            double r = (double)k / (NT / 2);
            h[n] = (float)(si * i0(10.0 * std::sqrt(std::max(0.0, 1 - r * r))) / i0(10.0));
            sum += h[n];
        }
        for (auto& v : h) { v = (float)(v / sum * L); }
    }
    const size_t nOut = (size_t)((double)y.size() * L / M);
    std::vector<cf> z(nOut);
    for (size_t o = 0; o < nOut; o++) {
        size_t up = o * M;   // index at 288 kS/s
        cf acc(0.0f, 0.0f);
        for (int t = 0; t < NT; t++) {
            long u = (long)up + t - NT / 2;
            if (u < 0 || u % L) { continue; }
            size_t i = (size_t)(u / L);
            if (i >= y.size()) { continue; }
            acc += y[i] * h[t];
        }
        z[o] = acc;
    }
    // ---- carrier at -3 kHz + 11 Hz, level, noise (SNR about 30 dB in the channel) ----
    std::mt19937 rng(20261018);
    std::normal_distribution<float> nd(0.0f, 1.0f);
    const double fs = 32000.0, f0 = -3000.0 + 11.0;
    double p = 0.0;
    for (auto& v : z) { p += std::norm(v); }
    p /= z.size();
    float amp = (float)(0.25 / std::sqrt(p));
    float sigma = (float)(0.25 * std::sqrt(std::pow(10.0, -30.0 / 10.0) * fs / 25000.0 / 2.0));
    for (size_t i = 0; i < z.size(); i++) {
        double a = 2 * PI_D * f0 * (double)i / fs;
        z[i] = z[i] * amp * cf((float)std::cos(a), (float)std::sin(a)) + cf(sigma * nd(rng), sigma * nd(rng));
    }
    size_t want = (size_t)(seconds * fs);
    if (z.size() > want) { z.resize(want); }
    writeWav(argv[1], z, 32000);
    fprintf(stderr, "wrote %s: %zu samples (%.2f s)\n", argv[1], z.size(), z.size() / fs);
    return 0;
}
