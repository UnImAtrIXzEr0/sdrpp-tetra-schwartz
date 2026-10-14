#pragma once
// Bridge between a blocking SDR++ stream and an audio device callback.
//
// The device callback asks for an arbitrary number of frames. Reading exactly
// one stream packet per callback (as the sinks used to do) consumes too little
// whenever the device period is larger than the packet, and the resulting
// back-pressure stalls the whole DSP chain up to the SDR source, which then
// drops IQ samples. Here a reader thread moves packets into a small ring and
// the callback always takes exactly what the device needs (silence on
// underrun). The reader blocks while the ring is full, so the consumption
// rate is the device rate and file playback stays paced.
#include <dsp/stream.h>
#include <dsp/types.h>
#include <algorithm>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

class StereoAudioRing {
public:
    ~StereoAudioRing() { stop(); }

    void start(dsp::stream<dsp::stereo_t>* in, size_t capacity) {
        stop();
        _in = in;
        {
            std::lock_guard<std::mutex> lck(_mtx);
            _buf.assign((std::max)(capacity, (size_t)256), dsp::stereo_t{ 0.0f, 0.0f });
            _r = _w = _fill = 0;
            _stop = false;
        }
        _thread = std::thread(&StereoAudioRing::worker, this);
    }

    void stop() {
        if (!_thread.joinable()) { return; }
        {
            std::lock_guard<std::mutex> lck(_mtx);
            _stop = true;
        }
        _cv.notify_all();
        _in->stopReader();
        _thread.join();
        _in->clearReadStop();
    }

    // Device side: never blocks. Fills 'n' frames, silence where no data.
    size_t pull(dsp::stereo_t* out, size_t n) {
        size_t k = 0;
        {
            std::lock_guard<std::mutex> lck(_mtx);
            k = (std::min)(n, _fill);
            for (size_t i = 0; i < k; i++) {
                out[i] = _buf[_r];
                _r = (_r + 1) % _buf.size();
            }
            _fill -= k;
        }
        if (k) { _cv.notify_all(); }
        if (k < n) { memset((void*)(out + k), 0, (n - k) * sizeof(dsp::stereo_t)); }
        return k;
    }

private:
    void worker() {
        while (true) {
            int count = _in->read();
            if (count < 0) { return; }
            const dsp::stereo_t* src = _in->readBuf;
            size_t done = 0;
            while (done < (size_t)count) {
                std::unique_lock<std::mutex> lck(_mtx);
                _cv.wait(lck, [this] { return _stop || _fill < _buf.size(); });
                if (_stop) {
                    lck.unlock();
                    _in->flush();
                    return;
                }
                size_t k = (std::min)((size_t)count - done, _buf.size() - _fill);
                for (size_t i = 0; i < k; i++) {
                    _buf[_w] = src[done + i];
                    _w = (_w + 1) % _buf.size();
                }
                _fill += k;
                done += k;
            }
            _in->flush();
        }
    }

    dsp::stream<dsp::stereo_t>* _in = nullptr;
    std::vector<dsp::stereo_t> _buf;
    size_t _r = 0, _w = 0, _fill = 0;
    bool _stop = false;
    std::mutex _mtx;
    std::condition_variable _cv;
    std::thread _thread;
};
