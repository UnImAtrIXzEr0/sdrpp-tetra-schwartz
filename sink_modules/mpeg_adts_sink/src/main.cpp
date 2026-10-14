#include <utils/networking.h>
#include <imgui.h>
#include <module.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <signal_path/sink.h>
#include <dsp/buffer/packer.h>
#include <dsp/convert/stereo_to_mono.h>
#include <dsp/sink/handler_sink.h>
#include <utils/flog.h>
#include <config.h>
#include <gui/style.h>
#include <core.h>
#include <lame.h>

#define CONCAT(a, b) ((std::string(a) + b).c_str())

SDRPP_MOD_INFO{
    /* Name:            */ "mpeg_adts_sink",
    /* Description:     */ "MPEG ADTS sink module for SDR++",
    /* Author:          */ "san",
    /* Version:         */ 0, 1, 0,
    /* Max instances    */ 1
};


ConfigManager config;

class MPEGADTSSink : SinkManager::Sink {
public:

    lame_t lame;
    std::vector<uint8_t> mp3_buffer;
    static constexpr int MAX_MP3_FRAME_SIZE = 2880; // Max size of an MP3 frame at 320kbps



    MPEGADTSSink(SinkManager::Stream* stream, std::string streamName) {
        _stream = stream;
        _streamName = streamName;

        // Load config
        config.acquire();
        if (!config.conf.contains(_streamName)) {
            config.conf[_streamName]["hostname"] = "localhost";
            config.conf[_streamName]["autostart"] = false;
            config.conf[_streamName]["port"] = 2020;
        }
        std::string host = config.conf[_streamName]["hostname"];
        strcpy(hostname, host.c_str());
        port = config.conf[_streamName]["port"];
        bool startNow = config.conf[_streamName]["autostart"];
        config.release(true);

        netBuf = new int16_t[STREAM_BUFFER_SIZE];

        packer.init(_stream->sinkOut, 512);
        s2m.init(&packer.out);
        monoSink.init(&s2m.out, monoHandler, this);

        _stream->setSampleRate(48000);

        // Start if needed
        if (startNow) { startNetwork(); }


        lame = lame_init();
        lame_set_in_samplerate(lame, 48000);
        lame_set_num_channels(lame, 1);
        lame_set_out_samplerate(lame, 48000);
        lame_set_brate(lame, 320);
        lame_set_mode(lame, MONO);
        lame_set_quality(lame, 0);  // 2 is a good quality setting (0 is best, 9 is worst)
        lame_init_params(lame);

        mp3_buffer.resize(2 * MAX_MP3_FRAME_SIZE);
    }



    ~MPEGADTSSink() {
        stopNetwork();
        delete[] netBuf;
    }

    void start() {
        if (running) {
            return;
        }
        doStart();
        running = true;
    }

    void stop() {
        if (!running) {
            return;
        }
        doStop();
        running = false;
    }

    void menuHandler() {
        float menuWidth = ImGui::GetContentRegionAvail().x;

        bool listening = (conn && conn->isOpen());

        if (listening) { style::beginDisabled(); }
        if (ImGui::InputText(CONCAT("##_network_sink_host_", _streamName), hostname, 1023)) {
            config.acquire();
            config.conf[_streamName]["hostname"] = hostname;
            config.release(true);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputInt(CONCAT("##_network_sink_port_", _streamName), &port, 0, 0)) {
            config.acquire();
            config.conf[_streamName]["port"] = port;
            config.release(true);
        }

        if (listening) { style::endDisabled(); }

        if (listening && ImGui::Button(CONCAT("Stop##_network_sink_stop_", _streamName), ImVec2(menuWidth, 0))) {
            stopNetwork();
            config.acquire();
            config.conf[_streamName]["autostart"] = false;
            config.release(true);
        }
        else if (!listening && ImGui::Button(CONCAT("Start##_network_sink_stop_", _streamName), ImVec2(menuWidth, 0))) {
            startNetwork();
            config.acquire();
            config.conf[_streamName]["autostart"] = true;
            config.release(true);
        }

        ImGui::TextUnformatted("Status:");
        ImGui::SameLine();
        if (conn && conn->isOpen()) {
            ImGui::TextColored(ImVec4(0.0, 1.0, 0.0, 1.0), "Connected");
        }
        else if (listening) {
            ImGui::TextColored(ImVec4(1.0, 1.0, 0.0, 1.0), "Not connected");
        }
        else {
            ImGui::TextUnformatted("Idle");
        }
    }

private:
    void doStart() {
        packer.start();
        flog::warn("Starting");
        s2m.start();
        monoSink.start();
    }

    void doStop() {
        packer.stop();
        s2m.stop();
        monoSink.stop();
    }

    void startNetwork() {
        // conn = net::connect(hostname, port);
    }

    void stopNetwork() {
        if (conn) { conn->close(); }
        conn = nullptr;
    }

    std::mutex outPacketsSizeMtx;
    std::vector<std::shared_ptr<std::vector<uint8_t>>> outPackets;

    FILE *out = nullptr;

    static void writeADTSHeader(uint8_t* header, int frame_size) {
        // ADTS header is 7 bytes
        header[0] = 0xFF;  // Sync word
        header[1] = 0xFB;  // MPEG-1, Layer III, no CRC
        header[2] = 0xE5;  // 320kbps, 48kHz

        // Frame length (including header)
        uint32_t full_frame_size = frame_size + 7;
        header[3] = ((full_frame_size & 0x1800) >> 11) | 0xC0;  // Two highest bits of frame length + two padding bits
        header[4] = (full_frame_size & 0x7F8) >> 3;  // Next 8 bits of frame length
        header[5] = ((full_frame_size & 0x7) << 5) | 0x00;  // Last 3 bits of frame length + 5 bits of buffer fullness

        // Buffer fullness (variable bit rate not used, so set to 0)
        header[6] = 0x00;
    }

    float scale;

    static void monoHandler(float* samples, int count, void* ctx) {
        MPEGADTSSink* _this = (MPEGADTSSink*)ctx;

        float maxx = 0;
        for(int q=0; q<count; q++) {
            maxx = std::max<float>(maxx, fabs(samples[q]));
        }
        if (maxx > _this->scale) {
            _this->scale = maxx;
        }
        for (int q = 0; q < count; q++) {
            samples[q] = samples[q] / _this->scale * 32767;
        }

        // Encode to MP3
        int mp3_size = lame_encode_buffer_float(_this->lame, samples, samples, count, _this->mp3_buffer.data(), _this->mp3_buffer.size());

        auto out = std::make_shared<std::vector<uint8_t>>(mp3_size);

        flog::info("Sending {} samples -> {}", count, mp3_size);
        if (mp3_size == 0) {
            return;
        }

        // Write ADTS-like header
        //writeADTSHeader(out.get()->data(), mp3_size);

        // Copy MP3 data after the header
        std::memcpy(out->data(), _this->mp3_buffer.data(), mp3_size);

        _this->outPacketsSizeMtx.lock();
        _this->outPackets.push_back(out);
        _this->outPacketsSizeMtx.unlock();

        if (!_this->out) {
            _this->out = fopen("out.mp3", "wb");
        }
        fwrite(out->data(), 1, out->size(), _this->out);
        fflush(_this->out);

    }



    SinkManager::Stream* _stream;
    dsp::buffer::Packer<dsp::stereo_t> packer;
    dsp::convert::StereoToMono s2m;
    dsp::sink::Handler<float> monoSink;

    std::string _streamName;

    int srId = 0;
    bool running = false;

    char hostname[1024];
    int port = 4242;

    std::string sampleRatesTxt;
    unsigned int sampleRate = 48000;

    int16_t* netBuf;

    net::Conn conn;
    std::mutex connMtx;
};

class MPEGADTSSinkModule : public ModuleManager::Instance {
public:
    MPEGADTSSinkModule(std::string name) {
        this->name = name;
        provider.create = create_sink;
        provider.ctx = this;

        sigpath::sinkManager.registerSinkProvider("MPEGADTS Network", provider);
    }

    ~MPEGADTSSinkModule() {
        // Unregister sink, this will automatically stop and delete all instances of the audio sink
        sigpath::sinkManager.unregisterSinkProvider("MPEGADTS Network");
    }

    void postInit() {}

    void enable() {
        enabled = true;
    }

    void disable() {
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

private:
    static SinkManager::Sink* create_sink(SinkManager::Stream* stream, std::string streamName, void* ctx) {
        return (SinkManager::Sink*)(new MPEGADTSSink(stream, streamName));
    }

    std::string name;
    bool enabled = true;
    SinkManager::SinkProvider provider;
};

MOD_EXPORT void _INIT_() {
    json def = json({});
    config.setPath(std::string(core::getRoot()) + "/mpeg_adts_network_sink_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT void* _CREATE_INSTANCE_(std::string name) {
    MPEGADTSSinkModule* instance = new MPEGADTSSinkModule(name);
    return instance;
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (MPEGADTSSinkModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}