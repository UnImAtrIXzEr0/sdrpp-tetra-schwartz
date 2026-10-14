#pragma once
#include "windowed_sinc.h"
#include "estimate_tap_count.h"
#include "../window/nuttall.h"

namespace dsp::taps {
    template<typename T>
    inline tap<T> lowPass0(double cutoff, double transWidth, double sampleRate, bool oddTapCount = false) {
        int count = estimateTapCount(transWidth, sampleRate);
        if (oddTapCount && !(count % 2)) { count++; }
        return windowedSinc<T>(count, cutoff, sampleRate, window::nuttall);
    }
    inline tap<float> lowPass(double cutoff, double transWidth, double sampleRate, bool oddTapCount = false) {
        return lowPass0<float>(cutoff, transWidth, sampleRate, oddTapCount);
    }

}