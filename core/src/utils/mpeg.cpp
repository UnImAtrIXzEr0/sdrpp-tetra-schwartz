
#include <vector>
#include <cstdint>
#include <cstring>

#include "mpeg.h"
#include "flog.h"

#ifdef __APPLE__
#include <CoreAudio/CoreAudio.h>
#include <AudioToolbox/AudioToolbox.h>
#include <CoreFoundation/CoreFoundation.h>
#endif

#ifdef __APPLE__
CFStringRef stdStringToCFString(const std::string& input) {
    return CFStringCreateWithCString(kCFAllocatorDefault, input.c_str(), kCFStringEncodingUTF8);
}
#endif

namespace mpeg {

std::pair<std::vector<uint8_t>, std::string> produce_speech_file_for_whisper(const std::vector<dsp::stereo_t>& in, int sampleRate) {
    std::vector<uint8_t> wavData;

    if (in.empty()) {
        return {wavData, "wav"};
    }

#ifdef __APPLE__
    // Convert stereo_t to mono float samples (average of L/R channels)
    std::vector<float> mono;
    mono.reserve(in.size());
    for (const auto& sample : in) {
        mono.push_back((sample.l + sample.r) * 0.5f);
    }

    // Create AudioBufferList
    AudioBufferList bufferList;
    bufferList.mNumberBuffers = 1;
    bufferList.mBuffers[0].mNumberChannels = 1;
    bufferList.mBuffers[0].mDataByteSize = static_cast<UInt32>(mono.size() * sizeof(float));
    bufferList.mBuffers[0].mData = const_cast<float*>(mono.data()); // Avoid const issue

    // Create ExtAudioFile for writing MP4
    AudioStreamBasicDescription asbd = {0};
    asbd.mSampleRate = sampleRate;
    asbd.mFormatID = kAudioFormatMPEG4AAC;
    asbd.mFormatFlags = kMPEG4Object_AAC_LC;
    asbd.mFramesPerPacket = 1024; // Typical AAC frame size
    asbd.mChannelsPerFrame = 1;   // Mono
    asbd.mBitsPerChannel = 0;     // Unused for AAC
    asbd.mBytesPerPacket = 0;     // Variable bitrate
    asbd.mBytesPerFrame = 0;      // Variable bitrate

    // Create client format (PCM float)
    AudioStreamBasicDescription clientFormat = {0};
    clientFormat.mSampleRate = sampleRate;
    clientFormat.mFormatID = kAudioFormatLinearPCM;
    clientFormat.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    clientFormat.mBitsPerChannel = 32;
    clientFormat.mChannelsPerFrame = 1;
    clientFormat.mBytesPerPacket = 4;
    clientFormat.mBytesPerFrame = 4;
    clientFormat.mFramesPerPacket = 1;

    std::string extension = "m4a";
    auto filePath = "temp_whisper." + extension;
    auto cfFilePath = stdStringToCFString(filePath);

    CFURLRef tempFileURL = CFURLCreateWithFileSystemPath(kCFAllocatorDefault, cfFilePath, kCFURLPOSIXPathStyle, false);
    if (!tempFileURL) {
        flog::error("Failed to create CFURL.");
        CFRelease(cfFilePath);
        return {wavData, extension};
    }

    ExtAudioFileRef audioFile;
    OSStatus status = ExtAudioFileCreateWithURL(tempFileURL, kAudioFileM4AType, &asbd, nullptr, kAudioFileFlags_EraseFile, &audioFile);
    if (status != noErr) {
        flog::error("Failed to create audio file: {}", status);
        CFRelease(cfFilePath);
        CFRelease(tempFileURL);
        return {wavData, extension};
    }

    // Set client format
    status = ExtAudioFileSetProperty(audioFile, kExtAudioFileProperty_ClientDataFormat, sizeof(clientFormat), &clientFormat);
    if (status != noErr) {
        flog::error("Failed to set client format: {}", status);
        ExtAudioFileDispose(audioFile);
        CFRelease(cfFilePath);
        CFRelease(tempFileURL);
        return {wavData, extension};
    }

    // Write audio data
    UInt32 numFrames = static_cast<UInt32>(mono.size());
    status = ExtAudioFileWrite(audioFile, numFrames, &bufferList);
    if (status != noErr) {
        flog::error("Failed to write audio data: {}", status);
        ExtAudioFileDispose(audioFile);
        CFRelease(cfFilePath);
        CFRelease(tempFileURL);
        return {wavData, extension};
    }

    // Dispose of audio file and read data into wavData
    ExtAudioFileDispose(audioFile);
    CFRelease(cfFilePath);
    CFRelease(tempFileURL);

    FILE* file = fopen(filePath.c_str(), "rb");
    if (file) {
        fseek(file, 0, SEEK_END);
        long size = ftell(file);
        fseek(file, 0, SEEK_SET);
        wavData.resize(size);
        fread(wavData.data(), 1, size, file);
        fclose(file);
    } else {
        flog::error("Failed to read file: {}", filePath);
    }

    return {wavData, extension};
#else
    return {wavData, "wav"};
#endif
}

} // namespace mpeg
