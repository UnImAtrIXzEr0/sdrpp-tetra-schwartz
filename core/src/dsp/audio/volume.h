#pragma once
#include "../processor.h"

// TODO: This block is useless and weird, get rid of it
namespace dsp::audio {
    class Volume : public Processor<stereo_t, stereo_t> {
        using base_type = Processor<stereo_t, stereo_t>;
    public:
        Volume() {}

        Volume(stream<stereo_t>* in, double volume, bool muted) { init(in, volume, muted); }

        void init(stream<stereo_t>* in, double volume, bool muted) {
            _volume = powf(volume, 2);
            _muted = muted;
            _tempMuted = false;
            out.origin = "volume.out";
            base_type::init(in);
        }

        void setVolume(double volume) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _volume = powf(volume, 2);
        }

        void setMuted(bool muted) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _muted = muted;
        }

        void setTempMuted(bool muted) {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            _tempMuted = muted;
        }

        // Mute state set by the user only (without the temporary mute used while
        // transmitting); this is what the GUI shows and what gets saved.
        bool getUserMuted() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            return _muted;
        }

        bool getMuted() {
            assert(base_type::_block_init);
            std::lock_guard<std::recursive_mutex> lck(base_type::ctrlMtx);
            return _muted || _tempMuted;
        }

        inline int process(int count, const stereo_t* in, stereo_t* out) {
            volk_32f_s32f_multiply_32f((float*)out, (float*)in, (_muted || _tempMuted) ? 0.0f : _volume, count * 2);
            return count;
        }

        virtual int run() {
//            flog::info("VolumeAdjust: reading from: {}", base_type::_in->origin);
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            base_type::_in->flush();
//            flog::info("VolumeAdjust: swapping to : {}", base_type::out.origin);
            if (!base_type::out.swap(count)) {
//                flog::info("VolumeAdjust: oops");
                return -1;
            }
//            flog::info("VolumeAdjust: wrote to : {}", base_type::out.origin);
            return count;
        }

    private:
        float _volume;
        bool _muted;
        bool _tempMuted;

    };
}