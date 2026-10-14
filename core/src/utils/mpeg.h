#pragma once
#include <string>
#include <vector>
#include <dsp/types.h>

namespace mpeg {
    // Convert stereo audio samples to mono audio file format for Whisper
    // Input: stereo samples, sample rate (default 48kHz)
    // Output: pair of audio file bytes and file extension
    std::pair<std::vector<uint8_t>, std::string> produce_speech_file_for_whisper(const std::vector<dsp::stereo_t>& in, int sampleRate = 48000);
}
