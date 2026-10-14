#pragma once

#include <memory>
#include <string>
#include <vector>
#include <imgui.h>
#include <dsp/types.h>

struct SubWaterfall {

    struct SubWaterfallPrivate;

    std::vector<float> peaks;

    std::shared_ptr<SubWaterfallPrivate> pvt;

    SubWaterfall(int sampleRate, int wfrange, const std::string &);
    ~SubWaterfall();
    void init();
    void draw();
    void setFreqVisible(bool visible);
    void addAudioSamples(dsp::stereo_t * samples, int count, int sampleRate);

};