#pragma once
#include "../processor.h"

namespace dsp::convert {
    class RealToComplex : public Processor<float, complex_t> {
        using base_type = Processor<float, complex_t>;
    public:
        RealToComplex() {}

        RealToComplex(stream<float>* in) { init(in); }

        ~RealToComplex() {
            if (!base_type::_block_init) { return; }
            base_type::stop();
        }

        void init(stream<float>* in) {
            base_type::init(in);
        }

        static float *allocNullBuffer() {
            auto b = buffer::alloc<float>(STREAM_BUFFER_SIZE);
            buffer::clear(b, STREAM_BUFFER_SIZE);
            return b;
        }

        inline static int process(int count, const float* in, complex_t* out) {
            static float *nullBuf = allocNullBuffer();
            volk_32f_x2_interleave_32fc((lv_32fc_t*)out, in, nullBuf, count);
            return count;
        }

        int run() {
            int count = base_type::_in->read();
            if (count < 0) { return -1; }

            process(count, base_type::_in->readBuf, base_type::out.writeBuf);

            base_type::_in->flush();
            if (!base_type::out.swap(count)) { return -1; }
            return count;
        }

    private:
        float* nullBuf;

    };
}