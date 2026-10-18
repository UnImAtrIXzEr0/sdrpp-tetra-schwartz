#pragma once
// Decimation by 2 for the shared analysis tap (no SDR++ dependencies, unit-testable).
#include <algorithm>
#include <cmath>
#include <complex>
#include <vector>

namespace tetra_tap {
    // decimate-by-2 filter (odd length, linear phase)
    inline std::vector<float> decimTaps() {
        const int N = 111;
        const double fs = 288000.0, fc = 72000.0, beta = 10.0;
        auto i0 = [](double x) {
            double s = 1.0, t = 1.0;
            for (int k = 1; k < 50; k++) { t *= (x / (2.0 * k)) * (x / (2.0 * k)); s += t; }
            return s;
        };
        std::vector<double> h(N);
        double sum = 0.0;
        const int m = (N - 1) / 2;
        for (int n = 0; n < N; n++) {
            int k = n - m;
            double x = 2.0 * fc / fs;
            double sinc = k == 0 ? x : std::sin(M_PI * x * k) / (M_PI * k);
            double r = (double)k / m;
            double w = i0(beta * std::sqrt((std::max)(0.0, 1.0 - r * r))) / i0(beta);
            h[n] = sinc * w;
            sum += h[n];
        }
        std::vector<float> out(N);
        for (int n = 0; n < N; n++) { out[n] = (float)(h[n] / sum); }
        return out;
    }

    // streaming decimator by 2 (keeps its history across blocks)
    class Decim2 {
    public:
        Decim2() : taps(decimTaps()), hist(taps.size() - 1, std::complex<float>(0.0f, 0.0f)) {}
        void reset() { std::fill(hist.begin(), hist.end(), std::complex<float>(0.0f, 0.0f)); phase = 0; }
        // appends the decimated samples of 'in' to 'out'
        void process(const std::complex<float>* in, int n, std::vector<std::complex<float>>& out) {
            const size_t L = taps.size();
            buf.resize(hist.size() + (size_t)n);
            std::copy(hist.begin(), hist.end(), buf.begin());
            std::copy(in, in + n, buf.begin() + hist.size());
            // output k uses buf[i .. i+L-1] for i = phase, phase+2, ...
            size_t i = (size_t)phase;
            for (; i + L <= buf.size(); i += 2) {
                std::complex<float> acc(0.0f, 0.0f);
                const std::complex<float>* p = &buf[i];
                for (size_t t = 0; t < L; t++) { acc += p[t] * taps[t]; }
                out.push_back(acc);
            }
            // keep the last L-1 samples, carry the phase
            size_t consumed = buf.size() - hist.size();
            phase = (int)(i - consumed);
            std::copy(buf.end() - (long)hist.size(), buf.end(), hist.begin());
        }
    private:
        std::vector<float> taps;
        std::vector<std::complex<float>> hist, buf;
        int phase = 0;
    };
}
