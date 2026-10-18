#pragma once
// Propagation diagram maths (no SDR++ dependencies, unit-testable):
// level samples along a drive, averaged over 50 m travelled (local mean with
// the fast fading removed - Lee: 20-40 wavelengths are enough, 50 m is about
// 70 wavelengths at 430 MHz), placed around the base station by distance and
// bearing.

#include <algorithm>
#include <cmath>
#include <vector>

namespace prop {
    constexpr double R_EARTH = 6371008.8;
    constexpr double DEG = 3.14159265358979323846 / 180.0;

    inline double distanceM(double lat1, double lon1, double lat2, double lon2) {
        double p1 = lat1 * DEG, p2 = lat2 * DEG, dp = (lat2 - lat1) * DEG, dl = (lon2 - lon1) * DEG;
        double a = std::sin(dp / 2) * std::sin(dp / 2) + std::cos(p1) * std::cos(p2) * std::sin(dl / 2) * std::sin(dl / 2);
        return 2.0 * R_EARTH * std::atan2(std::sqrt(a), std::sqrt((std::max)(0.0, 1.0 - a)));
    }
    // initial bearing from point 1 to point 2, degrees clockwise from north, 0..360
    inline double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
        double p1 = lat1 * DEG, p2 = lat2 * DEG, dl = (lon2 - lon1) * DEG;
        double y = std::sin(dl) * std::cos(p2);
        double x = std::cos(p1) * std::sin(p2) - std::sin(p1) * std::cos(p2) * std::cos(dl);
        double b = std::atan2(y, x) / DEG;
        return b < 0 ? b + 360.0 : b;
    }

    struct Point {
        double lat = 0, lon = 0;
        float levelDb = NAN;   // local mean (power average)
        float merDb = NAN;     // mean MER of the samples with MER
        int samples = 0;
    };

    // Collects samples and closes a bin every 'binM' metres travelled.
    class Binner {
    public:
        double binM = 50.0;
        double maxGapS = 5.0;      // longer without a fix or level: the bin is dropped
        std::vector<Point> points;

        void clear() { points.clear(); resetBin(); haveLast = false; }

        // t: seconds (monotonic); fix: position valid; level in dB (NAN: none)
        void add(double t, bool fix, double lat, double lon, float levelDb, float merDb) {
            if (!fix || std::isnan(levelDb)) {
                if (haveLast && t - lastT > maxGapS) { resetBin(); haveLast = false; }
                return;
            }
            if (haveLast && t - lastT > maxGapS) { resetBin(); haveLast = false; }
            if (haveLast) {
                double d = distanceM(lastLat, lastLon, lat, lon);
                if (d < 1000.0) { travelled += d; }   // a jump of the receiver: ignore the step
            }
            haveLast = true;
            lastT = t; lastLat = lat; lastLon = lon;
            sumLin += std::pow(10.0, levelDb / 10.0);
            sumLat += lat; sumLon += lon; n++;
            if (!std::isnan(merDb)) { sumMer += merDb; nMer++; }
            if (travelled >= binM && n >= 3) {
                Point p;
                p.lat = sumLat / n; p.lon = sumLon / n;
                p.levelDb = (float)(10.0 * std::log10(sumLin / n));
                p.merDb = nMer > 0 ? (float)(sumMer / nMer) : NAN;
                p.samples = n;
                points.push_back(p);
                resetBin();
            }
        }
        // metres of the bin in progress (for the display)
        double pending() const { return travelled; }

    private:
        void resetBin() { travelled = 0; sumLin = sumLat = sumLon = sumMer = 0; n = nMer = 0; }
        bool haveLast = false;
        double lastT = 0, lastLat = 0, lastLon = 0;
        double travelled = 0, sumLin = 0, sumLat = 0, sumLon = 0, sumMer = 0;
        int n = 0, nMer = 0;
    };

    // ring spacing: 1, 2 or 5 x 10^k so that 'maxM' needs at most 'rings' rings
    inline double niceStep(double maxM, int rings = 5) {
        double raw = (std::max)(maxM, 1.0) / rings;
        double e = std::pow(10.0, std::floor(std::log10(raw)));
        for (double m : { 1.0, 2.0, 5.0, 10.0 }) { if (m * e >= raw) { return m * e; } }
        return 10.0 * e;
    }
}
