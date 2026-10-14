#pragma once
// Decouples the decoded TETRA audio from the audio sink.
//
// SDR++ streams are blocking: if an audio sink consumes slower than the
// decoder produces (device clock, sink callback size mismatch, PipeWire
// quantum, ...), the back-pressure travels up through the demodulator and
// the VFO to the SDR source and the receiver loses IQ samples. For TETRA
// this means lost bursts and constant re-locks.
//
// This block never blocks its input: decoded audio goes into a bounded
// FIFO, a separate pump thread feeds the (blocking) audio path. When the
// sink is too slow, the oldest audio is dropped instead of RF samples.
#include <dsp/processor.h>
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

namespace dsp {
    class AudioDecoupler : public Processor<float, float> {
        using base_type = Processor<float, float>;
    public:
        AudioDecoupler() {}
        ~AudioDecoupler() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
        }

        void init(stream<float>* in, size_t maxSamples) {
            _max = (std::max)((size_t)1024, maxSamples);
            base_type::init(in);
        }

        uint64_t getDroppedSamples() { return _dropped.load(); }
        uint64_t getPassedSamples() { return _passed.load(); }

        int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }
            {
                std::lock_guard<std::mutex> lck(_fifoMtx);
                const float* src = base_type::_in->readBuf;
                _fifo.insert(_fifo.end(), src, src + count);
                if (_fifo.size() > _max) {
                    size_t drop = _fifo.size() - _max;
                    _fifo.erase(_fifo.begin(), _fifo.begin() + drop);
                    _dropped.fetch_add(drop, std::memory_order_relaxed);
                }
            }
            _fifoCv.notify_one();
            base_type::_in->flush();
            return count;
        }

    protected:
        void doStart() {
            {
                std::lock_guard<std::mutex> lck(_fifoMtx);
                _pumpStop = false;
            }
            _pumpThread = std::thread(&AudioDecoupler::pump, this);
            base_type::doStart();
        }

        void doStop() {
            {
                std::lock_guard<std::mutex> lck(_fifoMtx);
                _pumpStop = true;
            }
            _fifoCv.notify_all();
            base_type::out.stopWriter();
            if (_pumpThread.joinable()) { _pumpThread.join(); }
            base_type::doStop();   // stops the input reader, joins the worker, clears the stop flags
        }

    private:
        void pump() {
            const size_t chunk = 800;   // 100 ms at 8 kHz
            while (true) {
                size_t n = 0;
                {
                    std::unique_lock<std::mutex> lck(_fifoMtx);
                    _fifoCv.wait(lck, [this] { return _pumpStop || !_fifo.empty(); });
                    if (_pumpStop) { return; }
                    n = (std::min)(_fifo.size(), chunk);
                    std::copy(_fifo.begin(), _fifo.begin() + n, base_type::out.writeBuf);
                    _fifo.erase(_fifo.begin(), _fifo.begin() + n);
                }
                _passed.fetch_add(n, std::memory_order_relaxed);
                if (!base_type::out.swap((int)n)) { return; }
            }
        }

        std::deque<float> _fifo;
        std::mutex _fifoMtx;
        std::condition_variable _fifoCv;
        bool _pumpStop = false;
        size_t _max = 8000;
        std::thread _pumpThread;
        std::atomic<uint64_t> _dropped { 0 };
        std::atomic<uint64_t> _passed { 0 };
    };
}
