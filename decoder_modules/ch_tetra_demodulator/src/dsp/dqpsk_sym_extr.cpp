#include "dqpsk_sym_extr.h"

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
