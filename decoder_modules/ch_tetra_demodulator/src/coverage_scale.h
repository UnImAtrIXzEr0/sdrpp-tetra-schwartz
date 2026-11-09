#pragma once
// Colour scales of the coverage map, shared by the map on screen and the
// KMZ/KML/HTML export: fixed ranges so that drives are comparable,
// red (poor) -> yellow -> green (good), quality in three steps.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace cov {
    enum Metric { LEVEL = 0, MER = 1, BER = 2, QUALITY = 3 };

    struct Scale {
        std::string name, unit, title, loText, hiText;
        double lo = 0, hi = 1;
        bool logScale = false;
        bool steps = false;    // three colors only (quality)
        double norm(double v) const {
            double t;
            if (logScale) {
                double lv = std::log10((std::max)(v, 1e-7));
                t = (lv - std::log10(lo)) / (std::log10(hi) - std::log10(lo));
            }
            else {
                t = (v - lo) / (hi - lo);
            }
            return (std::min)(1.0, (std::max)(0.0, t));
        }
    };

    inline std::string fmtNum(double v) {
        char b[32];
        snprintf(b, sizeof(b), "%g", v);
        return b;
    }

    inline Scale scaleFor(int m, bool useDbm, float crcLimit) {
        Scale s;
        if (m == LEVEL) {
            if (useDbm) { s.lo = -115; s.hi = -65; s.unit = "dBm"; }
            else { s.lo = -95; s.hi = -45; s.unit = "dBFS"; }
            s.name = "Level";
            s.title = "Level (" + s.unit + ")";
            s.loText = fmtNum(s.lo);
            s.hiText = fmtNum(s.hi);
        }
        else if (m == MER) {
            s.lo = 5; s.hi = 30; s.unit = "dB"; s.name = "MER";
            s.title = "MER (dB)";
            s.loText = "5";
            s.hiText = "30";
        }
        else if (m == QUALITY) {
            s.lo = 0; s.hi = 1; s.unit = "";
            s.name = "Quality (CRC: red > " + fmtNum(10.0 * crcLimit) + " %, yellow, green <= " + fmtNum(crcLimit) + " %)";
            s.title = "Quality (CRC errors)";
            s.loText = "> " + fmtNum(10.0 * crcLimit) + " %";
            s.hiText = "\xE2\x89\xA4 " + fmtNum(crcLimit) + " %";
            s.steps = true;
        }
        else {
            s.lo = 1e-1; s.hi = 1e-5; s.unit = ""; s.name = "BER"; s.logScale = true;
            s.title = "BER";
            s.loText = "1e-1";
            s.hiText = "1e-5";
        }
        return s;
    }

    // CRC error share (%) -> 0 poor, 0.5 marginal, 1 good
    inline double quality(double crcPct, float crcLimit) {
        return crcPct <= crcLimit ? 1.0 : crcPct <= 10.0 * crcLimit ? 0.5 : 0.0;
    }

    // red -> yellow -> green
    inline void ramp(double t, int& r, int& g, int& b) {
        if (t < 0.5) { r = 230; g = (int)(40 + 400 * t); b = 40; }
        else { r = (int)(230 - 400 * (t - 0.5)); g = 230; b = 40; }
        r = (std::max)(0, (std::min)(255, r));
        g = (std::max)(0, (std::min)(255, g));
    }
}
