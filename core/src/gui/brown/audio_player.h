#pragma once

#include <imgui_internal.h>
#include <imgui.h>
#include <string>
#include <dsp/stream.h>
#include <dsp/routing/splitter.h>
#include <dsp/types.h>

struct AudioPlayer {
    std::string id;
    std::string error;
    float position;
    bool playing = false;
    int64_t dataPosition;
    dsp::routing::Splitter<dsp::stereo_t> splitter;
    dsp::stream<dsp::stereo_t> outStream;
    dsp::stream<dsp::stereo_t> splitterOut;
    std::vector<dsp::stereo_t> *data;
    std::vector<dsp::stereo_t> dataOwn;
    std::function<void()> onPlayStart;
    std::function<void()> onPlayEnd;
    AudioPlayer(const std::string &id) {
        this->id = id;
        position = 0;
        splitter.init(&outStream);
        splitter.bindStream(&splitterOut);
    }
    bool loadFile(const std::string &path);
    void setData(std::vector<dsp::stereo_t> *data, uint32_t sampleRate);
    void draw();
    void startPlaying();
    void stopPlaying();
    uint32_t sampleRate;
};