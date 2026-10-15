#include "dqpsk_sym_extr.h"

namespace dsp {
    // Solve the 3x3 normal equations of the I/Q model (Gauss-Jordan, pivoting),
    // publish image rejection and the phase / amplitude error split.
    void DQPSKSymbolExtractor::solveIqModel() {
        std::complex<double> m[3][4];
        for (int u = 0; u < 3; u++) {
            for (int v = 0; v < 3; v++) { m[u][v] = lsG[u][v]; }
            m[u][3] = lsH[u];
        }
        bool ok = true;
        for (int col = 0; col < 3 && ok; col++) {
            int piv = col;
            for (int r = col + 1; r < 3; r++) { if (std::abs(m[r][col]) > std::abs(m[piv][col])) { piv = r; } }
            if (std::abs(m[piv][col]) < 1e-12) { ok = false; break; }
            if (piv != col) { for (int k = 0; k < 4; k++) { std::swap(m[piv][k], m[col][k]); } }
            std::complex<double> inv = 1.0 / m[col][col];
            for (int k = col; k < 4; k++) { m[col][k] *= inv; }
            for (int r = 0; r < 3; r++) {
                if (r == col) { continue; }
                std::complex<double> f = m[r][col];
                for (int k = col; k < 4; k++) { m[r][k] -= f * m[col][k]; }
            }
        }
        if (ok && std::abs(m[0][3]) > 1e-9) {
            lsMu = m[0][3]; lsNu = m[1][3]; lsC = m[2][3];
            lsValid = true;
            double ratio = std::abs(lsNu) / std::abs(lsMu);
            irrDb.store(ratio > 1e-6 ? (float)(-20.0 * log10(ratio)) : 120.0f);
            // origin offset from the model: the modulation is removed by the
            // fit, so this is far more sensitive than the plain mean
            double co = std::abs(lsC) / std::abs(lsMu);
            iqOffsetDb.store(co > 1e-5 ? (float)(20.0 * log10(co)) : -100.0f);
            // and use it for the display / decision compensation as well
            prevOffRe = lsC.real();
            prevOffIm = lsC.imag();
            imbRatio.store((float)ratio);
        }
        if (splitCount > 100) {
            ampErrPct.store((float)(100.0 * sqrt(sumAmpErr2 / splitCount)));
            phaseErrDeg.store((float)(sqrt(sumPhErr2 / splitCount) * 180.0 / 3.14159265358979));
        }
        for (int u = 0; u < 3; u++) { for (int v = 0; v < 3; v++) { lsG[u][v] = 0.0; } lsH[u] = 0.0; }
        sumAmpErr2 = sumPhErr2 = 0.0;
        splitCount = 0;
    }
}


namespace dsp {
    int DQPSKSymbolExtractor::process(int count, const complex_t* in, uint8_t* out) {
        for(int i = 0; i < count; i++) {
            static const float rc[8] = { 1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f, 0.0f, 0.70710678f };
            static const float rs[8] = { 0.0f, 0.70710678f, 1.0f, 0.70710678f, 0.0f, -0.70710678f, -1.0f, -0.70710678f };
            const complex_t sym_in = in[i];
            // symbol with the previous window's I/Q offset removed (raw pi/4 frame)
            complex_t sym_corr;
            {
                float rre = sym_in.re * rc[rawRot] - sym_in.im * rs[rawRot] - (float)prevOffRe;
                float rim = sym_in.re * rs[rawRot] + sym_in.im * rc[rawRot] - (float)prevOffIm;
                sym_corr.re = rre * rc[rawRot] + rim * rs[rawRot];
                sym_corr.im = -rre * rs[rawRot] + rim * rc[rawRot];
            }
            complex_t sym_c = iqComp.load(std::memory_order_relaxed) ? sym_corr : sym_in;
            bool a = sym_c.im<0;
            bool b = sym_c.re<0;
            complex_t ideal_sym = {b?-0.7071f : 0.7071f, a?-0.7071f : 0.7071f};
            // float dist = fabsf(sqrtf(((ideal_sym.re-sym_c.re)*(ideal_sym.re-sym_c.re))+((ideal_sym.im-sym_c.im)*(ideal_sym.im-sym_c.im))));
            float dist = std::abs(ideal_sym.phase() - sym_c.phase());

            // MER/EVM: normalize by the mean magnitude of the block so the
            // AGC set point does not bias the result
            {
                float mag = sym_in.amplitude();
                sumMag += mag;
                // offset estimate always from the uncorrected symbols
                sumRawRe += sym_in.re * rc[rawRot] - sym_in.im * rs[rawRot];
                sumRawIm += sym_in.re * rs[rawRot] + sym_in.im * rc[rawRot];
                {
                    bool ca = sym_corr.im < 0, cb = sym_corr.re < 0;
                    float xr = sym_corr.re - (cb ? -0.7071f : 0.7071f) * lastScale;
                    float xi = sym_corr.im - (ca ? -0.7071f : 0.7071f) * lastScale;
                    sumErrEx += xr * xr + xi * xi;
                    // I/Q imbalance model in the raw (transmitter) frame:
                    //   r = mu*d + nu*conj(d) + c   (least squares over the block)
                    std::complex<double> d(cb ? -0.70710678 : 0.70710678, ca ? -0.70710678 : 0.70710678);
                    d *= std::complex<double>(rc[rawRot], rs[rawRot]);
                    std::complex<double> r(sym_in.re * rc[rawRot] - sym_in.im * rs[rawRot],
                                           sym_in.re * rs[rawRot] + sym_in.im * rc[rawRot]);
                    const std::complex<double> a[3] = { d, std::conj(d), 1.0 };
                    for (int u = 0; u < 3; u++) {
                        for (int v = 0; v < 3; v++) { lsG[u][v] += std::conj(a[u]) * a[v]; }
                        lsH[u] += std::conj(a[u]) * r;
                    }
                    // error split with the previous block's model
                    if (lsValid) {
                        std::complex<double> ideal = lsMu * d + lsNu * std::conj(d);
                        std::complex<double> q = r - lsC;
                        double mi = std::abs(ideal);
                        if (mi > 1e-9 && std::abs(q) > 1e-9) {
                            double ae = (std::abs(q) - mi) / mi;
                            double pe = std::arg(q * std::conj(ideal));
                            sumAmpErr2 += ae * ae;
                            sumPhErr2 += pe * pe;
                            splitCount++;
                        }
                    }
                }
                rawRot = (rawRot + 1) & 7;
                // MER/EVM as transmitted: uncorrected symbols, own decisions
                float er = sym_in.re - (sym_in.re < 0 ? -0.7071f : 0.7071f) * lastScale;
                float ei = sym_in.im - (sym_in.im < 0 ? -0.7071f : 0.7071f) * lastScale;
                float e2 = er * er + ei * ei;
                sumErr += e2;
                sumSig += (double)lastScale * lastScale;
                float pe = sqrtf(e2) / (lastScale > 1e-6f ? lastScale : 1e-6f);
                if (pe > peakErr) { peakErr = pe; }
                merCount++;
                if (merCount >= SYNC_DETECT_BUF) {
                    if (sumErr > 1e-12) {
                        merDb.store((float)(10.0 * log10(sumSig / sumErr)));
                        evmPct.store((float)(100.0 * sqrt(sumErr / sumSig)));
                    }
                    evmPeakPct.store(peakErr * 100.0f);
                    lastScale = (float)(sumMag / merCount);
                    {
                        double mr = sumRawRe / merCount, mi = sumRawIm / merCount;
                        double p = (mr * mr + mi * mi) / ((double)lastScale * lastScale + 1e-20);
                        iqOffsetDb.store(p > 1e-10 ? (float)(10.0 * log10(p)) : -100.0f);
                        merExclDb.store(sumErrEx > 1e-12 ? (float)(10.0 * log10(sumSig / sumErrEx)) : 0.0f);
                        prevOffRe = mr;
                        prevOffIm = mi;
                    }
                    solveIqModel();
                    sumRawRe = sumRawIm = 0.0;
                    sumErrEx = 0.0;
                    sumErr = sumSig = sumMag = 0.0;
                    peakErr = 0.0f;
                    merCount = 0;
                }
            }
            errorbuf[errorptr] = dist;
            errorptr++;
            if(errorptr >= SYNC_DETECT_BUF) {
                errorptr = 0;
            }
            errordisplayptr++;
            if(errordisplayptr >= SYNC_DETECT_DISPLAY) {
                float xerr = 0;
                for(int i = 0; i < SYNC_DETECT_BUF; i++) {
                    xerr+=errorbuf[i];
                }
                xerr /= (float)SYNC_DETECT_BUF;
                standarderr = xerr;
                if(xerr >= 0.35f) {
                    sync = false;
                } else {
                    sync = true;
                }
                errordisplayptr = 0;
            }
            uint8_t sym = ((a)<<1) | (a!=b); //This mapping is required to make substraction differential decoder work properly
            uint8_t phaseDiff = (sym - prev + 4) % 4;
            switch(phaseDiff) { //Remap phase diffs to actual tetra symbols(swap 0b10 and 0b11)
                case 0b00:
                    //0
                    out[i] = 0b00;
                    break;
                case 0b01:
                    //pi/2
                    out[i]=0b01;
                    break;
                case 0b10:
                    //pi
                    out[i]=0b11;
                    break;
                case 0b11:
                    //-pi/2
                    out[i]=0b10;
                    break;
            }
            prev = sym;
        }
        return count;
    }
}
