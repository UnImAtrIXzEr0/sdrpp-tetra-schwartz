#define GImGui (ImGui::GetCurrentContext())

#include <imgui.h>
#include <config.h>
#include <core.h>
#include <gui/style.h>
#include <gui/gui.h>
#include <signal_path/signal_path.h>
#include <module.h>
// #include <unistd.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <ctime>
#include <map>

#include <dsp/demod/psk.h>
#include <dsp/buffer/packer.h>
#include <dsp/routing/splitter.h>
#include <dsp/stream.h>
#include <dsp/convert/mono_to_stereo.h>

#include <gui/widgets/constellation_diagram.h>
#include <gui/widgets/file_select.h>
#include <gui/widgets/volume_meter.h>

#include <utils/flog.h>
#include <utils/net.h>

#include "dsp/bit_unpacker.h"
#include "dsp/dqpsk_sym_extr.h"
#include "dsp/pi4dqpsk.h"
#include "dsp/osmotetra_dec.h"
#include "dsp/audio_decoupler.h"
#include <gui/tuner.h>
#include "gui_widgets.h"
#include "tetra_colors.h"
#include "tetra_layout.h"
#include "analysis_tap.h"
#include "tx_analyzer.h"
#include "trend_view.h"
#include "carrier_monitor.h"
#include "band_monitor.h"
#include "open_ext.h"
#include "burst_analyzer.h"
#include "channel_analyzer.h"
#include "vem_analyzer.h"
#include "tetra_help.h"
#include <gui/touch.h>
#include "eval_view.h"
#include "alarms.h"
#include "report.h"
#include <version.h>
#include "measurements.h"
#include "subscribers.h"
#include "radioid.h"
#include "gps.h"
#include "propagation.h"
#include "map_export.h"
#include <signal_path/source_gain_interface.h>
#include <signal_path/source_freq_interface.h>
#include "const_view.h"
#include <filesystem>
#include <chrono>
#include <gui/widgets/folder_select.h>
#include <gui/gui.h>


#define CONCAT(a, b)    ((std::string(a) + b).c_str())

#define VFO_SAMPLERATE 72000
#define VFO_BANDWIDTH 45000
#define CLOCK_RECOVERY_BW 0.025f         // Changed from 0.00628f
#define CLOCK_RECOVERY_DAMPN_F 0.707f    // Kept (Critically Damped is perfect)
#define CLOCK_RECOVERY_REL_LIM 0.05f     // Changed from 0.02f
#define RRC_TAP_COUNT 65                 // Kept (Filter depth is fine)
#define RRC_ALPHA 0.35f                  // Kept (Strict TETRA standard)
#define AGC_RATE 0.05f                   // Changed from 0.02f
#define COSTAS_LOOP_BANDWIDTH 0.04f      // Changed from 0.01f
#define FLL_LOOP_BANDWIDTH 0.02f         // Changed from 0.006f

SDRPP_MOD_INFO {
    /* Name:            */ "ch_tetra_demodulator",
    /* Description:     */ "Tetra demodulator for SDR++(output can be fed to tetra-rx from osmo-tetra)",
    /* Author:          */ "cropinghigh",
    /* Version:         */ 0, 22, 1,
    /* Max instances    */ 1 // decoder core uses global state, a second instance would corrupt it
};

ConfigManager config;

class TetraDemodulatorModule : public ModuleManager::Instance {
public:
    TetraDemodulatorModule(std::string name) : logFolder("%ROOT%/tetra_logs") {
        this->name = name;

        // Load config
        config.acquire();
        if (!config.conf.contains(name) || !config.conf[name].contains("mode")) {
            config.conf[name]["mode"] = decoder_mode;
            config.conf[name]["hostname"] = "localhost";
            config.conf[name]["port"] = 8355;
            config.conf[name]["sending"] = false;
        }
        decoder_mode = config.conf[name]["mode"];
        // v0.20.1: NETSYMS is an advanced option. Left on without sending it
        // only switched the built-in decoder off - start with the decoder.
        if (!config.conf[name].contains("netsymsV2001")) {
            if (decoder_mode != 0 && !(config.conf[name].contains("sending") && config.conf[name]["sending"].get<bool>())) {
                decoder_mode = 0;
                config.conf[name]["mode"] = 0;
            }
            config.conf[name]["netsymsV2001"] = true;
        }
        snprintf(hostname, sizeof(hostname), "%s", std::string(config.conf[name]["hostname"]).c_str());
        port = config.conf[name]["port"];
        bool startNow = config.conf[name]["sending"];
        if (config.conf[name].contains("levelCalDb")) {
            levelCalDb = config.conf[name]["levelCalDb"];
        }
        if (config.conf[name].contains("levelCalGainDb") && config.conf[name]["levelCalGainDb"].is_number()) {
            levelCalGainDb = config.conf[name]["levelCalGainDb"];
        }
        levelCalEff = levelCalDb;
        if (config.conf[name].contains("freqRef") && config.conf[name]["freqRef"].is_number_integer()) {
            freqRef = std::clamp((int)config.conf[name]["freqRef"], 0, 2);
        }
        {
            // log folder: visible default on Linux (AppImage config is hidden),
            // created at start so logging works at once
            std::string def = tetraDefaultLogDir(logFolder.expandString("%ROOT%"));
            std::string lp = (config.conf[name].contains("logPath") && config.conf[name]["logPath"].is_string()) ? config.conf[name]["logPath"].get<std::string>() : def;
            if (!config.conf[name].contains("logPathV21")) {
                if (lp == "%ROOT%/tetra_logs") { lp = def; }
                config.conf[name]["logPathV21"] = true;
            }
            config.conf[name]["logPath"] = lp;
            std::error_code ec;
            std::filesystem::create_directories(logFolder.expandString(lp), ec);
            logFolder.setPath(lp);
            trendFile.setPath(lp);
        }
        if (config.conf[name].contains("constellation8")) {
            constellation8 = config.conf[name]["constellation8"];
        }
        if (config.conf[name].contains("constDetached")) { constDetached = config.conf[name]["constDetached"]; }
        if (config.conf[name].contains("subsDetached")) { subsDetached = config.conf[name]["subsDetached"]; }
        loadSectionOrder();
        if (config.conf[name].contains("netDetached")) { netDetached = config.conf[name]["netDetached"]; }
        if (config.conf[name].contains("txaDetached")) { txaDetached = config.conf[name]["txaDetached"]; }
        if (config.conf[name].contains("txaRun")) { txaRun = config.conf[name]["txaRun"]; }
        if (config.conf[name].contains("txaFft")) { txaFftIdx = std::clamp((int)config.conf[name]["txaFft"], 0, 2); }
        if (config.conf[name].contains("txaAvg")) { txaAvg = std::clamp((int)config.conf[name]["txaAvg"], 1, 100); }
        if (config.conf[name].contains("txaLimits") && config.conf[name]["txaLimits"].is_array() && config.conf[name]["txaLimits"].size() == 3) {
            for (int i = 0; i < 3; i++) { txaLimit[i] = config.conf[name]["txaLimits"][i]; }
        }
        if (config.conf[name].contains("compOriginOffset")) { compOriginOffset = config.conf[name]["compOriginOffset"]; }
        if (config.conf[name].contains("afc")) { afcEnabled = config.conf[name]["afc"]; }
        if (config.conf[name].contains("autoGain")) { autoGain = config.conf[name]["autoGain"]; }
        if (config.conf[name].contains("gainMode") && config.conf[name]["gainMode"].is_number_integer()) { gainMode = std::clamp((int)config.conf[name]["gainMode"], 0, 2); }
        if (config.conf[name].contains("trendDetached")) { trendDetached = config.conf[name]["trendDetached"]; }
        if (config.conf[name].contains("measDetached")) { measDetached = config.conf[name]["measDetached"]; }
        if (config.conf[name].contains("gpsDetached")) { gpsDetached = config.conf[name]["gpsDetached"]; }
        if (config.conf[name].contains("carrDetached")) { carrDetached = config.conf[name]["carrDetached"]; }
        if (config.conf[name].contains("burstDetached")) { burstDetached = config.conf[name]["burstDetached"]; }
        if (config.conf[name].contains("trendSpan")) { trendView.spanIdx = std::clamp((int)config.conf[name]["trendSpan"], 0, TetraTrendView::SPAN_N - 1); }
        if (config.conf[name].contains("trendShow") && config.conf[name]["trendShow"].is_array()) {
            for (int i = 0; i < TetraTrendView::P_COUNT && i < (int)config.conf[name]["trendShow"].size(); i++) { trendView.show[i] = config.conf[name]["trendShow"][i]; }
        }
        if (config.conf[name].contains("burst") && config.conf[name]["burst"].is_object()) {
            json& b = config.conf[name]["burst"];
            if (b.contains("run")) { burstRun = b["run"]; }
            if (b.contains("source")) { burstSt.source = std::clamp((int)b["source"], 0, 2); }
            if (b.contains("mhz")) { burstSt.manualMHz = b["mhz"]; }
            if (b.contains("avg")) { burstSt.averages = std::clamp((int)b["avg"], 1, 500); }
            if (b.contains("ripple")) { burstSt.maskRippleDb = b["ripple"]; }
            if (b.contains("off")) { burstSt.maskOffDbc = b["off"]; }
            if (b.contains("ramp")) { burstSt.maskRampSym = b["ramp"]; }
        }
        if (config.conf[name].contains("vemRun")) { vemRun = config.conf[name]["vemRun"]; }
        if (config.conf[name].contains("chanRaster")) { chanRaster = config.conf[name]["chanRaster"]; }
        if (config.conf[name].contains("vfoLocked")) { vfoLocked = config.conf[name]["vfoLocked"]; }
        if (config.conf[name].contains("snapCarrier")) { snapCarrier = config.conf[name]["snapCarrier"]; }
        startViewDone = config.conf[name].contains("startViewV21");
        if (config.conf[name].contains("mp") && config.conf[name]["mp"].is_object()) {
            json& mp = config.conf[name]["mp"];
            if (mp.contains("run")) { mpRun = mp["run"]; }
            if (mp.contains("mem")) { mpMem = std::clamp((int)mp["mem"], 0, 3); }
            if (mp.contains("detached")) { mpDetached = mp["detached"]; }
        }
        if (config.conf[name].contains("crcLimit")) { evalView.crcLimit = config.conf[name]["crcLimit"]; }
        if (config.conf[name].contains("burst") && config.conf[name]["burst"].is_object()) {
            json& b = config.conf[name]["burst"];
            // from v0.22.2 the uplink is measured alone by default (timing vs. the
            // downlink only on request): older saved values are not taken over
            if (b.contains("timing") && b.contains("ulOnlyV222")) { burstSt.timing = b["timing"]; }
            if (b.contains("linStepDb")) { burstSt.linStepDb = std::clamp((float)b["linStepDb"], 3.0f, 30.0f); }
            if (b.contains("transients")) { burstSt.transients = b["transients"]; }
            if (b.contains("stdMask")) { burstSt.stdMask = b["stdMask"]; }
            if (b.contains("station")) { burstSt.station = std::clamp((int)b["station"], 0, 1); }
            if (b.contains("msClass4")) { burstSt.msClass4 = b["msClass4"]; }
            if (b.contains("msPowerKnown")) { burstSt.msPowerKnown = b["msPowerKnown"]; }
            if (b.contains("msPowerDbm")) { burstSt.msPowerDbm = b["msPowerDbm"]; }
            if (b.contains("floorSpan")) { burstSt.floorSpan = b["floorSpan"]; }
        }
        if (config.conf[name].contains("alarms") && config.conf[name]["alarms"].is_object()) {
            json& a = config.conf[name]["alarms"];
            auto gb = [&](const char* k, bool& v) { if (a.contains(k) && a[k].is_boolean()) { v = a[k]; } };
            auto gf = [&](const char* k, float& v) { if (a.contains(k) && a[k].is_number()) { v = a[k]; } };
            gb("enabled", alarmSt.enabled); gb("levelOn", alarmSt.levelOn); gf("levelBelow", alarmSt.levelBelow);
            gb("merOn", alarmSt.merOn); gf("merBelow", alarmSt.merBelow); gb("crcOn", alarmSt.crcOn); gf("crcAbove", alarmSt.crcAbove);
            gb("syncOn", alarmSt.syncOn); gb("ulOn", alarmSt.ulOn); gf("ulAbove", alarmSt.ulAbove);
            gb("beep", alarmSt.beep); gb("autoLog", alarmSt.autoLog); gb("lossOn", alarmSt.lossOn);
            if (a.contains("holdS") && a["holdS"].is_number()) { alarmSt.holdS = a["holdS"]; }
        }
        if (config.conf[name].contains("reportNotes") && config.conf[name]["reportNotes"].is_string()) {
            snprintf(reportNotes, sizeof(reportNotes), "%s", config.conf[name]["reportNotes"].get<std::string>().c_str());
        }
        json carrierList = json::array();
        if (config.conf[name].contains("carriers") && config.conf[name]["carriers"].is_array()) { carrierList = config.conf[name]["carriers"]; }
        if (config.conf[name].contains("gpsPort")) { gpsPort = config.conf[name]["gpsPort"]; }
        if (config.conf[name].contains("gpsBaud")) { gpsBaud = config.conf[name]["gpsBaud"]; }
        if (config.conf[name].contains("mapMetric")) { mapMetric = config.conf[name]["mapMetric"]; }
        loadTiles();
        if (config.conf[name].contains("prop") && config.conf[name]["prop"].is_object()) {
            json& pp = config.conf[name]["prop"];
            if (pp.contains("bsValid")) { propView.bsValid = pp["bsValid"]; }
            if (pp.contains("bsLat")) { propView.bsLat = pp["bsLat"]; }
            if (pp.contains("bsLon")) { propView.bsLon = pp["bsLon"]; }
            if (pp.contains("metric")) { propView.metric = std::clamp((int)pp["metric"], 0, 1); }
        }
        bool gpsStartNow = config.conf[name].contains("gpsEnabled") && config.conf[name]["gpsEnabled"];
        if (config.conf[name].contains("hamMode")) {
            hamMode = config.conf[name]["hamMode"];
        }
        // v0.2/0.3 kept an edited duplex table in the config: migrate it into the ham file
        json legacyTable;
        if (config.conf[name].contains("duplexTable")) {
            legacyTable = config.conf[name]["duplexTable"];
            config.conf[name].erase("duplexTable");
        }
        config.release(true);

        loadHamTable(legacyTable);
        applyDuplexMode();
        radioid.setDirectory(rootDir());
        radioid.loadAsync();
        gpsPorts = NmeaGps::listPorts();
        if (gpsStartNow && !gpsPort.empty()) { gps.start(gpsPort, gpsBaud); }

        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, VFO_BANDWIDTH, VFO_SAMPLERATE, VFO_BANDWIDTH, VFO_BANDWIDTH, true);

        //Clock recov coeffs
        float recov_bandwidth = CLOCK_RECOVERY_BW;
        float recov_dampningFactor = CLOCK_RECOVERY_DAMPN_F;
        float recov_denominator = (1.0f + 2.0*recov_dampningFactor*recov_bandwidth + recov_bandwidth*recov_bandwidth);
        float recov_mu = (4.0f * recov_dampningFactor * recov_bandwidth) / recov_denominator;
        float recov_omega = (4.0f * recov_bandwidth * recov_bandwidth) / recov_denominator;

        mainDemodulator.init(vfo->output, 18000, VFO_SAMPLERATE, RRC_TAP_COUNT, RRC_ALPHA, AGC_RATE, COSTAS_LOOP_BANDWIDTH, FLL_LOOP_BANDWIDTH, recov_omega, recov_mu, CLOCK_RECOVERY_REL_LIM);
        {
            // further carriers use the same demodulator settings
            TetraDemodParams dp;
            dp.sampleRate = VFO_SAMPLERATE; dp.bandwidth = VFO_BANDWIDTH; dp.rrcTaps = RRC_TAP_COUNT; dp.rrcAlpha = RRC_ALPHA;
            dp.agcRate = AGC_RATE; dp.costasBw = COSTAS_LOOP_BANDWIDTH; dp.fllBw = FLL_LOOP_BANDWIDTH;
            dp.omega = recov_omega; dp.mu = recov_mu; dp.relLim = CLOCK_RECOVERY_REL_LIM;
            carrierMon.setParams(dp);
            carrierMon.setBaseName(name);
            for (auto& c : carrierList) {
                if (c.is_object() && c.contains("freq")) { carrierMon.add(c.contains("label") ? c["label"].get<std::string>() : std::string(), c["freq"].get<double>()); }
            }
            carrierMon.takeChanged();
            burstAn.setName(name + " bursts");
            anaTap.setName(name + " analysis");
            chanAn.setTap(&anaTap);
            vemAn.setTap(&anaTap);
            txa.setTap(&anaTap);
            fftRedrawHandler.handler = fftRedraw;
            fftRedrawHandler.ctx = this;
            gui::waterfall.onFFTRedraw.bindHandler(&fftRedrawHandler);
        }
        constDiagSplitter.init(&mainDemodulator.out);
        constDiagSplitter.bindStream(&constDiagStream);
        constDiagSplitter.bindStream(&demodStream);
        constDiagReshaper.init(&constDiagStream, 1024, 0);
        constDiagSink.init(&constDiagReshaper.out, _constDiagSinkHandler, this);
        symbolExtractor.init(&demodStream);
        bitsUnpacker.init(&symbolExtractor.out);

        // Initialize demodSink with NULL input - set dynamically in setMode()
        // to avoid two readers on bitsUnpacker.out simultaneously
        demodSink.init(nullptr, _demodSinkHandler, this);

        osmotetradecoder.init(&symbolExtractor.out);
        osmotetradecoder.setAddressCallback(&TetraSubscriberTable::onAddress, &subscribers);
        osmotetradecoder.setL3Callback(&TetraSubscriberTable::onL3, &subscribers);
        
        // Audio must never stall the demodulator (see dsp/audio_decoupler.h)
        audioDecoupler.init(&osmotetradecoder.out, 4000);
        resamp.init(&audioDecoupler.out, 8000.0, audioSampleRate);
        outconv.init(&resamp.out);

        // Initialize the sink
        srChangeHandler.ctx = this;
        srChangeHandler.handler = sampleRateChangeHandler;
        stream.init(&srChangeHandler, audioSampleRate);
        stream.setInput(&outconv.out);
        sigpath::sinkManager.registerStream(name, &stream);

        mainDemodulator.start();
        constDiagSplitter.start();
        constDiagReshaper.start();
        constDiagSink.start();
        symbolExtractor.start();
        setMode();
        audioDecoupler.start();
        resamp.start();
        outconv.start();
        stream.start();
        startMeasurements();
        gui::menu.registerEntry(name, menuHandler, this, this);
        windowDrawHandler.ctx = this;
        windowDrawHandler.handler = [](ImGuiContext* gctx, void* ctx) {
            auto* m = (TetraDemodulatorModule*)ctx;
            m->txaLoop();
            m->extraLoops();
            m->controlLoops();
            m->drawFloatingWindows();
        };
        gui::mainWindow.onWaterfallDrawn.bindHandler(&windowDrawHandler);

        if(startNow) {
            startNetwork();
        }
    }

    ~TetraDemodulatorModule() {
        removePanels();
        gui::waterfall.onFFTRedraw.unbindHandler(&fftRedrawHandler);
        meas.stop();
        txa.stop();
        carrierMon.stopAll();
        carrierMon.stopLog();
        burstAn.stop();
        chanAn.stop();
        vemAn.stop();
        if(isEnabled()) {
            disable();
        }
        gui::mainWindow.onWaterfallDrawn.unbindHandler(&windowDrawHandler);
        gui::menu.removeEntry(name);
        sigpath::sinkManager.unregisterStream(name);
    }

    void postInit() {}

    void enable() {
        if (enabled) { return; }

        vfo = sigpath::vfoManager.createVFO(name, ImGui::WaterfallVFO::REF_CENTER, 0, VFO_BANDWIDTH, VFO_SAMPLERATE, VFO_BANDWIDTH, VFO_BANDWIDTH, true);
        if (!vfo) {
            flog::error("TETRA: createVFO failed (name already in use?)");
            return;
        }

        float recov_bandwidth = CLOCK_RECOVERY_BW;
        float recov_dampningFactor = CLOCK_RECOVERY_DAMPN_F;
        float recov_denominator = (1.0f + 2.0*recov_dampningFactor*recov_bandwidth + recov_bandwidth*recov_bandwidth);
        float recov_mu = (4.0f * recov_dampningFactor * recov_bandwidth) / recov_denominator;
        float recov_omega = (4.0f * recov_bandwidth * recov_bandwidth) / recov_denominator;

        mainDemodulator.setInput(vfo->output);
        constDiagSplitter.setInput(&mainDemodulator.out);
        constDiagReshaper.setInput(&constDiagStream);
        symbolExtractor.setInput(&demodStream);

        // Start blocks from downstream to upstream.
        // resamp/outconv/stream are NOT stopped in disable(), so they idle
        // and resume automatically when upstream data flows again.
        symbolExtractor.start();
        constDiagSink.start();
        constDiagReshaper.start();
        constDiagSplitter.start();
        mainDemodulator.start();

        enabled = true;
        setMode();
        startMeasurements();
    }

    void disable() {
        if (!enabled) { return; }
        removePanels();
        bandMon.loop(false);
        if (SourceGainInterface* gi = findGainInterface()) { gi->gainLockedExt.store(false); }

        // Stop the measurement thread first: it reads the blocks and the VFO
        meas.stop();
        txa.stop();
        carrierMon.stopAll();
        carrierMon.stopLog();
        burstAn.stop();
        chanAn.stop();
        vemAn.stop();

        // Stop upstream first to drain data flow into the chain
        mainDemodulator.stop();
        constDiagSplitter.stop();
        constDiagReshaper.stop();
        constDiagSink.stop();
        symbolExtractor.stop();

        // Stop both mode endpoints and the optional unpacker path.
        osmotetradecoder.stop();
        osmotetradecoder.setInput(nullptr);
        demodSink.stop();
        demodSink.setInput(nullptr);
        bitsUnpacker.stop();

        // DON'T stop resamp/outconv/stream - the SinkManager merger has a detached
        // reader thread on outconv.out. Calling doStop() on outconv would stopWriter()
        // on outconv.out, killing the merger reader permanently. Let them idle:
        // when upstream is stopped, they block on read() with no data. On re-enable,
        // upstream restarts and they resume naturally.

        sigpath::vfoManager.deleteVFO(vfo);
        vfo = nullptr;
        enabled = false;
    }

    bool isEnabled() {
        return enabled;
    }

    // Automation channel
    std::string handleDebugCommand(const std::string& cmd, const std::string& args) override {
        if (cmd == "get_status") {
            int dec_st = osmotetradecoder.getRxState();
            std::string lockState = (dec_st == 0) ? "unlocked" : ((dec_st == 2) ? "locked" : "know_next_start");
            std::string status = "{\"decoder_state\": \"" + lockState + "\", "
                "\"sync\": " + std::string(symbolExtractor.sync ? "true" : "false") + ", "
                "\"signal_quality\": " + std::to_string(1.0f - symbolExtractor.standarderr) + ", "
                "\"mer_db\": " + std::to_string(symbolExtractor.getMerDb()) + ", "
                "\"carrier_offset_hz\": " + std::to_string(mainDemodulator.getCarrierOffsetHz()) + ", "
                "\"level_dbfs\": " + std::to_string(mainDemodulator.getLevelDbfs()) + ", "
                "\"version\": \"" TETRA_PLUGIN_VERSION "\", "
                "\"mode\": " + std::to_string(decoder_mode);
            if (decoder_mode == 0 && enabled) {
                status += ", \"hyperframe\": " + std::to_string(osmotetradecoder.getCurrHyperframe());
                status += ", \"multiframe\": " + std::to_string(osmotetradecoder.getCurrMultiframe());
                status += ", \"frame\": " + std::to_string(osmotetradecoder.getCurrFrame());
                status += ", \"mcc\": " + std::to_string(osmotetradecoder.getMcc());
                status += ", \"mnc\": " + std::to_string(osmotetradecoder.getMnc());
                status += std::string(", \"voice_service\": ") + (osmotetradecoder.getVoiceService() ? "true" : "false");
            }
            status += "}";
            return status;
        }
        if (cmd == "set_mode") {
            try {
                int mode = std::stoi(args);
                if (mode == 0 || mode == 1) {
                    decoder_mode = mode;
                    setMode();
                    return "{\"status\": \"ok\", \"mode\": " + std::to_string(mode) + "}";
                }
            } catch (...) {}
            return "{\"error\": \"invalid mode: " + args + "\"}";
        }
        // ---- additional automation for the edition's tests ----
        if (cmd == "set_offset") {
            try {
                double off = std::stod(args);
                if (vfo) { sigpath::vfoManager.setOffset(name, off); }
                return "{\"status\": \"ok\"}";
            } catch (...) {}
            return "{\"error\": \"invalid offset\"}";
        }
        if (cmd == "set_txa") {
            txaRun = (args == "1" || args == "true");
            return std::string("{\"status\": \"ok\", \"txa\": ") + (txaRun ? "true" : "false") + "}";
        }
        if (cmd == "set_windows") {
            // "1" = all own windows, "0" = none
            bool on = (args == "1" || args == "true");
            constDetached = subsDetached = netDetached = txaDetached = on;
            trendDetached = carrDetached = burstDetached = mpDetached = on;
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "set_layout") {
            // layout preset 0 classic, 1 measuring station, 2 drive test (applied in the GUI thread)
            layoutReq.store(std::clamp(atoi(args.c_str()), 0, 3));
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "set_bursts") {
            // burst analysis on the TETRA VFO frequency ("1") or off ("0")
            burstRun = (args == "1" || args == "true");
            burstSt.source = 2;
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "add_carrier") {
            // absolute frequency in Hz; added from the GUI thread
            pendingCarrierHz.store(atof(args.c_str()));
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "only_window") {
            // "name x y w h": show only this own window at a fixed place
            // (const, stations, network, txa, trends, carriers, bursts, multipath)
            static const char* names[8] = { "const", "stations", "network", "txa", "trends", "carriers", "bursts", "multipath" };
            char nm[32] = "";
            float r[4] = { 380, 40, 820, 760 };
            sscanf(args.c_str(), "%31s %f %f %f %f", nm, &r[0], &r[1], &r[2], &r[3]);
            int idx = -1;
            for (int i = 0; i < 8; i++) { if (!strcmp(nm, names[i])) { idx = i; } }
            if (idx < 0) { return "{\"error\": \"unknown window\"}"; }
            std::lock_guard<std::mutex> lck(dbgMtx);
            soloReq = idx;
            for (int i = 0; i < 4; i++) { soloRect[i] = r[i]; }
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "set_multipath") {
            mpRun = (args == "1" || args == "true");
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "set_burst_freq") {
            // manual burst frequency in Hz, timing against the TETRA VFO
            burstSt.source = 1;
            burstSt.manualMHz = atof(args.c_str()) / 1e6;
            burstSt.timing = true;
            burstSt.transients = true;
            burstRun = true;
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "set_alarm_test") {
            // level alarm that triggers at once (automation tests)
            alarmSt.enabled = true;
            alarmSt.levelOn = true;
            alarmSt.levelBelow = 100.0f;
            alarmSt.holdS = 1;
            alarmSt.autoLog = false;
            alarmSt.beep = false;
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "load_trend_csv") {
            // open a CSV log in the trends view (documentation, tests)
            std::lock_guard<std::mutex> lck(trendReqMtx);
            trendLoadReq = args;
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "set_trend_tab") {
            std::lock_guard<std::mutex> lck(trendReqMtx);
            trendTabReq = std::clamp(atoi(args.c_str()), 0, 2);
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "make_report") {
            reportReq.store(true);   // made in the GUI thread
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "get_report") {
            std::lock_guard<std::mutex> lck(reportMtx);
            std::string r = reportResult;
            for (auto& ch : r) { if (ch == '"' || ch == '\\') { ch = '/'; } }
            return "{\"path\": \"" + r + "\"}";
        }
        if (cmd == "set_afc") {
            afcEnabled = (args == "1" || args == "true");
            return "{\"status\": \"ok\"}";
        }
        if (cmd == "get_measurements") {
            TetraMeasSnapshot m = meas.get();
            TxAnalysisResult r = txa.get();
            TetraBurstResult br = burstAn.get();
            int carrCnt, carrRun, carrSync;
            std::string bstat;
            {
                std::lock_guard<std::mutex> lck(dbgMtx);
                carrCnt = dbgCarr[0]; carrRun = dbgCarr[1]; carrSync = dbgCarr[2];
                bstat = burstStatus;
            }
            uint32_t tsTraffic = 0, tsAll = 0;
            for (int t = 0; t < 4; t++) { for (int c = 0; c < 4; c++) { tsAll += m.dlUseTotal[t][c]; } tsTraffic += m.dlUseTotal[t][3]; }
            TetraChannelResult mpr = chanAn.get();
            TetraVemResult vr = vemAn.get();
            int alarmTotal;
            bool alarmActive;
            {
                std::lock_guard<std::mutex> lck(dbgMtx);
                alarmTotal = dbgAlarmTotal;
                alarmActive = dbgAlarmActive;
            }
            char b[4096];
            snprintf(b, sizeof(b),
                     "{\"valid\": %s, \"sync\": %s, \"rx_state\": %d, \"mer_db\": %.2f, \"evm_pct\": %.2f, "
                     "\"iq_offset_db\": %.2f, \"irr_db\": %.2f, \"phase_err_deg\": %.3f, \"amp_err_pct\": %.3f, "
                     "\"input_rate\": %.1f, \"input_rate_ppm\": %.1f, \"bursts\": %u, \"resync_slip\": %u, "
                     "\"carrier_offset_hz\": %.2f, \"audio_dropped_s\": %.2f, \"vfo_offset_hz\": %.1f, "
                     "\"txa_valid\": %s, \"txa_chan_dbfs\": %.2f, \"txa_obw_hz\": %.1f, \"txa_bw26_hz\": %.1f, "
                     "\"txa_acp25\": [%.2f, %.2f], \"txa_acp_floor25\": [%.2f, %.2f], \"txa_leak_dbc\": %.2f, "
                     "\"stations\": %d, \"traffic\": %d, \"center_hz\": %.0f, \"in_samplerate\": %.0f, "
                     "\"burst_state\": %d, \"burst_continuous\": %s, \"burst_status\": \"%s\", \"carriers\": %d, \"carriers_running\": %d, \"carriers_sync\": %d, "
                     "\"aach_count\": %u, \"aach_traffic\": %u, \"trend_points\": %d, "
                     "\"burst_n_full\": %llu, \"burst_n_half\": %llu, \"burst_len_sym\": %.2f, \"burst_rise_sym\": %.2f, \"burst_fall_sym\": %.2f, "
                     "\"burst_ripple_pos_db\": %.2f, \"burst_ripple_neg_db\": %.2f, \"burst_off_dbc\": %.1f, \"burst_floor_dbc\": %.1f, "
                     "\"burst_gap_dbfs\": %.1f, \"burst_timing_sym\": %.3f, \"burst_timing_std\": %.3f, \"burst_timing_n\": %d, \"burst_slot_quality\": %.1f, "
                     "\"burst_trans_up_p25\": %.1f, \"burst_trans_mid_p25\": %.1f, \"burst_trans_n\": %d, "
                     "\"mp_valid\": %s, \"mp_slots\": %d, \"mp_quality\": %.1f, \"mp_echo_us\": %.1f, \"mp_echo_db\": %.1f, \"mp_spread_us\": %.1f, "
                     "\"alarms_total\": %d, \"alarm_active\": %s, \"band_valid\": %s, \"band_peak_dbfs\": %.1f, \"band_snr_db\": %.1f, \"sections\": %d, \"burst_desense_db\": %.2f, \"burst_desense_n\": %d, \"freq_ref\": %d, \"vem_grid\": %s, \"vem_n\": %d, \"vem_measured\": %llu, \"vem_rejected\": %llu, \"vem_rms_pct\": %.3f, \"vem_rms_max_pct\": %.3f, \"vem_peak_pct\": %.2f, \"vem_c0_pct\": %.3f}",
                     m.valid ? "true" : "false", m.symSync ? "true" : "false", m.rxState, m.merDb, m.evmPct,
                     m.iqOffsetDb, symbolExtractor.getImageRejectionDb(), symbolExtractor.getPhaseErrDeg(), symbolExtractor.getAmpErrPct(),
                     m.inputRate, m.inputRatePpm, m.bursts, m.resyncSlip,
                     m.carrierOffsetHz, (double)audioDecoupler.getDroppedSamples() / 8000.0, vfo ? vfo->getOffset() : 0.0,
                     r.valid ? "true" : "false", r.chanPowerDbfs, r.obwHz, r.bw26Hz,
                     r.acpDbc[2], r.acpDbc[3], r.acpFloorDbc[2], r.acpFloorDbc[3], r.carrierLeakDbc,
                     (int)subscribers.snapshot().size(), (int)subscribers.trafficSnapshot().size(),
                     gui::waterfall.getCenterFrequency(), sigpath::iqFrontEnd.getSampleRate(),
                     br.state, br.continuous ? "true" : "false", bstat.c_str(), carrCnt, carrRun, carrSync,
                     tsAll, tsTraffic, (int)meas.trend().copy(0).size(),
                     (unsigned long long)br.nFull, (unsigned long long)br.nHalf, nz(br.lengthSym), nz(br.riseSym), nz(br.fallSym),
                     nz(br.ripplePosDb), nz(br.rippleNegDb), nz(br.offBeforeDbc), nz(br.floorDbc),
                     nz(br.gapDbfs), nz(br.timingMeanSym), nz(br.timingStdSym), br.timingN, (double)burstAn.slotQuality(),
                     br.transValid ? (double)br.transDbc[0][3] : -999.0, br.transValid ? (double)br.transDbc[1][3] : -999.0, br.transN,
                     mpr.valid ? "true" : "false", mpr.slots, (double)mpr.quality, nz(mpr.mp.echoDelayUs), nz(mpr.mp.echoLevelDb), nz(mpr.mp.extraSpreadUs),
                     alarmTotal, alarmActive ? "true" : "false",
                     bandRes.valid ? "true" : "false", nz(bandRes.peakDbfs), nz(bandRes.snrDb), (int)secOrder.size(), nz(br.desenseDb), br.desenseN, freqRef,
                     vr.gridValid ? "true" : "false", vr.n, (unsigned long long)vr.measured, (unsigned long long)vr.rejected,
                     vr.n ? vr.rmsMean * 100.0 : -999.0, vr.n ? vr.rmsMax * 100.0 : -999.0, vr.n ? vr.peakMax * 100.0 : -999.0, vr.n ? vr.c0Mean * 100.0 : -999.0);
            return b;
        }
        return "{\"error\": \"unknown command: " + cmd + "\"}";
    }

private:

    void startMeasurements() {
        meas.setGps(&gps);
        meas.setLevelCal(levelCalEff);
        meas.setStationSource(&subscribers,
            [this](uint32_t ssi) -> std::string {
                RadioIdEntry e;
                if (!radioid.lookup(ssi, e)) { return ";"; }
                std::string info = e.info;
                for (auto& c : info) { if (c == ';') { c = ','; } }
                return e.callsign + ";" + info;
            },
            [this]() -> bool { return hamMode; });
        meas.start(&mainDemodulator, &symbolExtractor, &osmotetradecoder,
            [this]() -> double {
                if (!vfo) { return 0.0; }
                return gui::waterfall.getCenterFrequency() + vfo->getOffset();
            },
            [this]() -> bool { return decoder_mode == 0; });
    }

    static void drawMeasurements(TetraDemodulatorModule* _this, float menuWidth) {
        TetraMeasSnapshot m = _this->meas.get();
        ImVec4 good = tc::good();
        ImVec4 warn = tc::warn();
        ImVec4 bad = tc::bad();
        ImVec4 val = tc::val();

        // Level
        tw::Text("Level:"); tetra_help::tip(tetra_help::LEVEL); tw::SameNext();
        if (m.valid) {
            if (_this->levelCalDb != 0.0f) {
                tw::TextColored(val, "%6.1f dBm", m.levelDbfs + _this->levelCalEff); tw::SameTextDisabled("(%.1f dBFS)", m.levelDbfs);
                if (std::fabs(_this->levelCalEff - _this->levelCalDb) > 0.05f) {
                    tw::SameTextDisabled("gain-corrected %+.1f dB", _this->levelCalEff - _this->levelCalDb);
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("The RF gain differs from the gain at calibration; the level is\n"
                                          "corrected by the difference from the SDR's gain table (about +-1 dB).");
                    }
                }
            } else {
                tw::TextColored(val, "%6.1f dBFS", m.levelDbfs);
            }
        } else { tw::TextDisabled("--"); }
        // fixed row (rows must not appear/disappear, the lines below would jump)
        tw::Text("RF status:"); tetra_help::tip(tetra_help::RF_STATUS); tw::SameNext();
        if (!m.valid) { tw::TextDisabled("--"); }
        else if (m.peakDbfs > -3.0f) { tw::TextColored(bad, "ADC OVERLOAD (peak %.1f dBFS) - reduce gain", m.peakDbfs); }
        else if (_this->bandRes.valid && _this->bandRes.peakDbfs > -3.0f) { tw::TextColored(bad, "ADC OVERLOAD by a signal in the band (%.1f dBFS)", _this->bandRes.peakDbfs); }
        else if (m.levelDbfs > -15.0f) { tw::TextColored(warn, "level high (peak %.1f dBFS)", m.peakDbfs); }
        else if (m.levelDbfs < -45.0f) { tw::TextColored(warn, "level low (peak %.1f dBFS)", m.peakDbfs); }
        else { tw::TextColored(good, "OK (peak %.1f dBFS)", m.peakDbfs); }

        // MER / EVM
        tw::Text("MER:"); tetra_help::tip(tetra_help::MER); tw::SameNext();
        if (m.valid && m.symSync) {
            tw::TextColored(m.merDb >= 20.0f ? good : (m.merDb >= 12.0f ? warn : bad), "%5.1f dB", m.merDb); tw::SameText(" decoder EVM"); tw::SameTextColored(val, "%4.1f %%", m.evmPct); tw::SameText(" peak"); tw::SameTextColored(val, "%4.0f %%", m.evmPeakPct);
        } else { tw::TextDisabled("-- (no symbol sync)"); }
        ImGui::SetNextItemWidth(menuWidth);
        float merFrac = m.symSync ? std::clamp(m.merDb / 30.0f, 0.0f, 1.0f) : 0.0f;
        char merTxt[32];
        snprintf(merTxt, sizeof(merTxt), "%.1f dB", m.symSync ? m.merDb : 0.0f);
        ImGui::ProgressBar(merFrac, ImVec2(menuWidth, 0), merTxt);
        tw::Text("I/Q offset:"); tw::SameNext();
        if (!(m.valid && m.symSync)) { tw::TextDisabled("--"); }
        else {
            tw::TextColored(m.iqOffsetDb <= -30.0f ? good : (m.iqOffsetDb <= -20.0f ? warn : bad), "%5.1f dB", m.iqOffsetDb);
            tw::SameText(" MER w/o offset"); tw::SameTextColored(val, "%4.1f dB", m.merExclDb);
            if (ImGui::IsItemHovered() || ImGui::IsItemHovered()) {
                ImGui::SetTooltip("I/Q (origin) offset = unmodulated carrier component.\n"
                                  "Shifts every point cloud by the same vector.\n"
                                  "Cause: transmitter LO leakage, or the SDR's DC offset if the SDR\n"
                                  "is tuned exactly to the carrier. Test: move the SDR center away\n"
                                  "(VFO offset / Offset Tuning). If the offset stays, it is the transmitter.");
            }
        }

        tw::Text("I/Q imbalance:"); tw::SameNext();
        if (!(m.valid && m.symSync)) { tw::TextDisabled("--"); }
        else {
            float irr = _this->symbolExtractor.getImageRejectionDb();
            double k = _this->symbolExtractor.getImbalanceRatio();
            tw::TextColored(irr >= 30.0f ? good : (irr >= 20.0f ? warn : bad), "IRR %4.1f dB", irr);
            tw::SameNext();
            double gdb = (k < 0.99) ? 20.0 * std::log10((1.0 + k) / (1.0 - k)) : 99.0;
            double pdeg = 2.0 * std::asin((std::min)(k, 1.0)) * 180.0 / 3.14159265;
            tw::TextDisabled("(= %.2f dB or %.1f deg)", gdb, pdeg);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Image rejection of the transmitter's I/Q modulator, from a fit of\n"
                                  "r = mu*d + nu*conj(d) + c to the symbols. Shown as an equivalent pure\n"
                                  "gain imbalance or pure quadrature error. Typical SDR transmitters\n"
                                  "without calibration: 25-35 dB. Shows as two different point cloud sizes\n"
                                  "and contributes to EVM and to unequal lower/upper shoulders.");
            }
        }
        tw::Text("Error split:"); tw::SameNext();
        if (!(m.valid && m.symSync)) { tw::TextDisabled("--"); }
        else {
            float ph = _this->symbolExtractor.getPhaseErrDeg();
            float am = _this->symbolExtractor.getAmpErrPct();
            tw::TextColored(val, "phase %.2f deg", ph); tw::SameTextColored(val, "amplitude %.2f %%", am); tw::SameNext();
            double rel = (am > 1e-3f) ? (ph * 3.14159265 / 180.0) / (am / 100.0) : 0.0;
            const char* why = rel > 2.0 ? "phase noise" : (rel < 0.5 ? "amplitude (AM)" : "noise-like");
            tw::TextDisabled("-> %s", why);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("RMS phase and amplitude error of the symbols around the fitted model.\n"
                                  "Both similar: additive noise (low level, interference).\n"
                                  "Phase clearly larger: phase noise / jitter of an oscillator.\n"
                                  "Amplitude clearly larger: compression, AM, level ripple.");
            }
        }

        // Frequency / clock
        tw::Text("Carrier offset:"); tw::SameNext();
        if (m.valid && m.symSync) {
            // EN 300 394-1 7.3.2.2: BS +-0.2 ppm up to 520 MHz, +-0.1 ppm above.
            // Judged only with a reference: B.1.2.1 asks +-0.02 ppm for the test receiver.
            float lim = m.refFreqHz > 520e6 ? 0.1f : 0.2f;
            tw::TextColored(val, "%+7.1f Hz", m.carrierOffsetHz);
            if (_this->freqRef == 0) { tw::SameTextColored(val, "(%+.3f ppm)", m.carrierPpm); tw::SameTextDisabled("indicative"); }
            else { tw::SameTextColored(std::fabs(m.carrierPpm) <= lim ? good : bad, "(%+.3f ppm)", m.carrierPpm); }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Carrier frequency error of the received cell (downlink).\n"
                                  "Nominal: %s %.4f MHz. Includes the SDR's own frequency error.\n"
                                  "EN 300 394-1 7.3.2: BS within +-%.1f ppm (MS +-100 Hz).\n"
                                  "B.1.2.1: the test receiver itself must be within +-0.02 ppm -\n"
                                  "only reachable with an external 10 MHz reference or a fresh\n"
                                  "calibration. Without one the value is indicative only.",
                                  m.refFromSysinfo ? "SYSINFO DL" : "VFO", m.refFreqHz / 1e6, lim);
            }
        } else { tw::TextDisabled("--"); }
        tw::Text("Frequency reference:"); tw::SameNext();
        ImGui::SetNextItemWidth((std::min)(menuWidth - ImGui::GetCursorPosX(), 190.0f * style::uiScale));
        if (ImGui::Combo(CONCAT("##_tetra_fref_", _this->name), &_this->freqRef, "SDR internal\0External 10 MHz\0Calibrated (XO corr.)\0")) {
            config.acquire();
            config.conf[_this->name]["freqRef"] = _this->freqRef;
            config.release(true);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Which reference the SDR runs on. Only with an external 10 MHz\n"
                              "reference (or a calibration against one) is the carrier error\n"
                              "judged against the ETSI limit; otherwise it is marked indicative.\n"
                              "Set the SDR itself as well: USRP (B2xx) Source -> Clock: external;\n"
                              "Pluto+: reference input is a firmware setting of the device.");
        }
        tw::Text("Symbol clock:"); tw::SameNext();
        if (m.valid && m.symSync) {
            tw::TextColored(val, "%+7.1f ppm", m.symbolRatePpm);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Symbol rate vs. 18 ksym/s, measured against the SDR clock.\nAveraged over ~2 s, includes SDR clock error.");
            }
        } else { tw::TextDisabled("--"); }
        tw::Text("Input rate:"); tw::SameNext();
        if (m.inputRate <= 0.0) { tw::TextDisabled("--"); }
        else {
            // the PC clock is the reference: some 10 ppm are clock differences;
            // a deficit clearly beyond the uncertainty means lost samples
            double tol = 3.0 * m.inputRateUncPpm + 50.0;
            bool loss = m.inputRatePpm < -tol;
            bool warnSt = !loss && std::fabs(m.inputRatePpm) > tol;
            tw::TextColored(loss ? bad : (warnSt ? warn : good), "%.0f S/s (%+.0f +-%.0f ppm)", m.inputRate, m.inputRatePpm, m.inputRateUncPpm);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Samples per second arriving at the TETRA demodulator (nominal 72000),\n"
                                  "least-squares rate over the last %.0f s against the PC clock.\n"
                                  "+-: statistical uncertainty (SDRs deliver samples in blocks).\n"
                                  "Red = clearly fewer samples than nominal: samples are lost (USB, CPU,\n"
                                  "audio device). Cross-check: 'Re-locks with timing slip' must not grow.",
                                  m.inputRateWindowS);
            }
        }
        tw::Text("Audio drops:"); tw::SameNext();
        {
            double droppedS = (double)_this->audioDecoupler.getDroppedSamples() / 8000.0;
            if (droppedS <= 0.0) { tw::TextColored(good, "none"); }
            else { tw::TextColored(warn, "%.1f s (audio sink too slow)", droppedS); }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Decoded TETRA audio discarded because the audio sink did not take it in time.\n"
                                  "The audio path is decoupled: a slow sink costs audio, never RF samples.");
            }
        }
        tw::Text("Carrier vs. SYSINFO:"); tw::SameNext();
        if (!m.absErrValid) { tw::TextDisabled("-- (needs decoder lock)"); }
        else {
            float ppm = (float)(m.absErrHz / (m.refFreqHz > 1e6 ? m.refFreqHz : 1e6) * 1e6);
            tw::TextColored(std::fabs(ppm) <= 0.2f ? good : (std::fabs(ppm) <= 1.0f ? warn : bad), "%+7.1f Hz (%+.3f ppm)", m.absErrHz, ppm);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Received carrier (VFO + offset) minus the DL frequency announced in SYSINFO.\n"
                                  "Independent of AFC. Contains BS error + SDR error; with a GPS-locked SDR\n"
                                  "this is the base station's frequency error (ETSI: +-0.2 ppm).");
            }
            SourceFreqCorrInterface* fc = _this->findFreqInterface();
            if (fc && fc->corrAvailable()) {
                if (tw::SameButtonOrFull(CONCAT("Calibrate SDR##_tetra_cal_", _this->name))) {
                    double refHz = m.refFreqHz > 1e6 ? m.refFreqHz : 1e6;
                    double oldPpm = fc->corrPpm();
                    // carrier too high by e ppm -> SDR oscillator runs slow by e ppm
                    double newPpm = oldPpm - m.absErrHz / refHz * 1e6;
                    double res = fc->corrResolutionPpm();
                    if (res > 0) { newPpm = std::round(newPpm / res) * res; }
                    fc->setCorrPpm(newPpm);
                    char b[128];
                    snprintf(b, sizeof(b), "SDR calibrated: correction %+.3f -> %+.3f ppm (error was %+.1f Hz)", oldPpm, newPpm, m.absErrHz);
                    _this->meas.addEvent(b);
                }
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Sets the SDR's oscillator correction so that this cell's carrier matches\n"
                                      "its SYSINFO frequency (current correction %+.3f ppm).\n"
                                      "Only use a cell whose frequency you trust (GPS-locked base station, or\n"
                                      "measured before with a reference-locked SDR). Let the SDR warm up first.",
                                      fc->corrPpm());
                }
            }
        }
        _this->drawAfcGain(menuWidth, m);

        if (_this->decoder_mode == 0) {
            // Bit errors in known bits (training sequences, FCF, tail bits)
            tw::Text("Raw BER:"); tw::SameNext();
            if (m.ber >= 0.0) {
                tw::TextColored(m.ber < 1e-3 ? good : (m.ber < 1e-2 ? warn : bad), "%.1e", m.ber); tw::SameText(" total"); tw::SameTextColored(val, "%.1e", m.berTotal);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Bit error ratio in known bits before channel decoding\n(training sequences, frequency correction field, tail bits).");
                }
            } else { tw::TextDisabled("--"); }
            tw::Text("Bursts/s:"); tw::SameTextColored(val, "%u", m.bursts); tw::SameText(" with errors"); tw::SameTextColored(m.tolerated ? warn : val, "%u", m.tolerated); tw::SameText(" sync lost"); tw::SameTextColored(m.syncLostTotal ? warn : val, "%u", m.syncLostTotal);
            {
                tw::Text("Re-locks:"); tw::SameTextColored(m.resyncSlip ? bad : val, "%u with timing slip", m.resyncSlip); tw::SameTextColored(val, " %u without", m.resyncNoSlip);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("After a sync loss the new burst timing is compared with the old one.\n"
                                      "Timing slip = symbols were lost or inserted in the sample stream\n"
                                      "(USB drops, CPU overload, SDR glitch e.g. during a gain change).\n"
                                      "No slip = the burst itself was corrupted (signal: fading, interference).");
                }
            }

            static const char* chNames[4] = { "BSCH ", "SB2  ", "Half ", "SCH/F" };
            if (ImGui::BeginTable(CONCAT("##_tetra_crc_", _this->name), 3, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
                ImGui::TableSetupColumn("CRC", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 4);
                ImGui::TableSetupColumn("1 s", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableSetupColumn("Total", ImGuiTableColumnFlags_WidthStretch);
                ImGui::TableHeadersRow();
                for (int i = 0; i < 4; i++) {
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted(chNames[i]);
                    ImGui::TableSetColumnIndex(1);
                    uint32_t n = m.crcOk[i] + m.crcFail[i];
                    if (n) {
                        float f = 100.0f * m.crcFail[i] / n;
                        tw::TextColored(f == 0.0f ? good : (f < 5.0f ? warn : bad), "%u/%u fail", m.crcFail[i], n);
                    } else { tw::TextDisabled("--"); }
                    ImGui::TableSetColumnIndex(2);
                    uint32_t nt = m.crcOkTotal[i] + m.crcFailTotal[i];
                    if (nt) {
                        tw::Text("%.2f %% fail", 100.0 * m.crcFailTotal[i] / nt);
                    } else { tw::TextDisabled("--"); }
                }
                ImGui::EndTable();
            }
        }

        if (ImGui::TreeNode(CONCAT("Events##_tetra_events_", _this->name))) {
            auto ev = _this->meas.getEvents();
            if (ImGui::SmallButton(CONCAT("Clear##_tetra_evclr_", _this->name))) { _this->meas.clearEvents(); ev.clear(); }
            if (ev.empty()) { tw::TextDisabled("no events"); }
            int shown = 0;
            for (auto it = ev.rbegin(); it != ev.rend() && shown < 30; ++it, ++shown) {
                struct tm lt;
#ifdef _WIN32
                localtime_s(&lt, &it->t);
#else
                localtime_r(&it->t, &lt);
#endif
                char ts[16];
                strftime(ts, sizeof(ts), "%H:%M:%S", &lt);
                bool isLoss = it->text.rfind("SYNC", 0) == 0 || it->text.find("slip ") != std::string::npos;
                tw::TextDisabled("%s", ts); tw::SameNext();
                if (isLoss) { tw::TextColored(warn, "%s", it->text.c_str()); }
                else { ImGui::TextUnformatted(it->text.c_str()); }
            }
            ImGui::TreePop();
        }
        if (ImGui::Button(CONCAT("Reset totals##_tetra_rst_", _this->name), ImVec2(menuWidth, 0))) {
            _this->meas.resetTotals();
        }

        // Level calibration
        tw::Text("Level cal. (dB):"); ImGui::SameLine();
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputFloat(CONCAT("##_tetra_lcal_", _this->name), &_this->levelCalDb, 0.0f, 0.0f, "%.1f")) {
            config.acquire();
            // remember the gain the calibration is made at (correction later)
            SourceGainInterface* cg = _this->findGainInterface();
            _this->levelCalGainDb = (cg && cg->gainControllable()) ? (float)cg->gainStepDb(cg->gainStep()) : NAN;
            if (std::isnan(_this->levelCalGainDb)) { config.conf[_this->name].erase("levelCalGainDb"); }
            else { config.conf[_this->name]["levelCalGainDb"] = _this->levelCalGainDb; }
            config.release(true);
            _this->levelCalEff = _this->levelCalDb;
            _this->meas.setLevelCal(_this->levelCalDb);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("dBm = dBFS + offset. Determine with a signal generator for your SDR\n"
                              "and frequency. The RF gain at the time of entry is remembered: later gain\n"
                              "changes are corrected from the SDR's gain table (about +-1 dB). 0 = dBFS.");
        }
        if (_this->levelCalDb != 0.0f && std::isnan(_this->levelCalGainDb) && _this->findGainInterface()) {
            tw::TextColored(tc::warn(), "Please enter the calibration again (gain at calibration unknown)");
        }

        // CSV logging
        bool logging = _this->meas.isLogging();
        if (logging) { style::beginDisabled(); }
        if (_this->logFolder.render("##_tetra_logdir_" + _this->name)) {
            if (_this->logFolder.pathIsValid()) {
                config.acquire();
                config.conf[_this->name]["logPath"] = _this->logFolder.path;
                config.release(true);
            }
        }
        if (logging) { style::endDisabled(); }
        if (!logging) {
            bool canLog = _this->logFolder.pathIsValid();
            if (!canLog) { style::beginDisabled(); }
            if (ImGui::Button(CONCAT("Start CSV log##_tetra_log_", _this->name), ImVec2(menuWidth, 0))) {
                std::string err;
                if (!_this->meas.startLog(_this->logFolder.expandString(_this->logFolder.path), err)) {
                    _this->logError = err;
                    flog::error("TETRA: {}", err);
                } else {
                    _this->logError.clear();
                    std::string lp = _this->meas.getLogPath();
                    if (_this->carrierMon.count() > 0 && lp.size() > 4) {
                        _this->carrierMon.startLog(lp.substr(0, lp.size() - 4) + "_carriers.csv", _this->levelCalEff);
                    }
                }
            }
            if (!canLog) { style::endDisabled(); }
        } else {
            if (ImGui::Button(CONCAT("Stop CSV log##_tetra_log_", _this->name), ImVec2(menuWidth, 0))) {
                _this->meas.stopLog();
                _this->carrierMon.stopLog();
            }
            tw::TextDisabled("%s", _this->meas.getLogPath().c_str());
        }
        if (!_this->logError.empty()) {
            tw::TextColored(bad, "%s", _this->logError.c_str());
        }
    }

    void startNetwork() {
        stopNetwork();
        try {
            conn = net::openudp(hostname, port);
        } catch (std::runtime_error& e) {
            flog::error("Network error: %s\n", e.what());
        }
    }

    void stopNetwork() {
        if (conn) { conn->close(); }
    }

    void setMode() {
        // Keep symbolExtractor.out single-consumer: osmo-tetra reads dibits directly,
        // NETSYMS uses the unpacked bit stream.
        osmotetradecoder.stop();
        osmotetradecoder.setInput(nullptr);
        demodSink.stop();
        demodSink.setInput(nullptr);
        bitsUnpacker.stop();

        if(decoder_mode == 0) {
            // osmo-tetra mode consumes dibits from symbolExtractor.out.
            osmotetradecoder.setInput(&symbolExtractor.out);
            osmotetradecoder.start();
        } else {
            bitsUnpacker.start();
            //network syms mode: use demodSink
            demodSink.setInput(&bitsUnpacker.out);
            demodSink.start();
        }
        config.acquire();
        config.conf[name]["mode"] = decoder_mode;
        config.release(true);
    }

    static void menuHandler(void* ctx) {
        TetraDemodulatorModule* _this = (TetraDemodulatorModule*)ctx;
        float menuWidth = ImGui::GetContentRegionAvail().x;

        if(!_this->enabled) {
            style::beginDisabled();
        }

        tw::TextDisabled("TETRA demodulator v" TETRA_PLUGIN_VERSION);
        if (_this->alarmSt.enabled && _this->alarms.any()) {
            tw::TextColored(tc::bad(), "ALARM: %s", _this->alarms.activeText().c_str());
        }
        // own audio volume / mute, independent of the Radio module
        tw::Text("Audio:"); ImGui::SameLine();
        sigpath::sinkManager.showVolumeSlider(_this->name, "##_tetra_vol_", menuWidth - ImGui::GetCursorPosX(), 0, 2, false);
        if (ImGui::RadioButton(CONCAT("ETSI##_tetra_mode_etsi_", _this->name), !_this->hamMode) && _this->hamMode) {
            _this->hamMode = false;
            _this->applyDuplexMode();
            config.acquire();
            config.conf[_this->name]["hamMode"] = false;
            config.release(true);
        }
        tw::SameLineFor(tw::checkboxWidth("Amateur radio"));
        if (ImGui::RadioButton(CONCAT("Amateur radio##_tetra_mode_ham_", _this->name), _this->hamMode) && !_this->hamMode) {
            _this->hamMode = true;
            _this->applyDuplexMode();
            config.acquire();
            config.conf[_this->name]["hamMode"] = true;
            config.release(true);
        }
        if (tw::SameSmallButton(CONCAT("Layout...##_tetra_layout_", _this->name))) { ImGui::OpenPopup(CONCAT("##_tetra_layout_pop_", _this->name)); }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Layout presets: sections as tiles in a column next to the waterfall"); }
        if (ImGui::BeginPopup(CONCAT("##_tetra_layout_pop_", _this->name))) {
            if (ImGui::MenuItem("Measuring station")) { _this->applyLayout(1); }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Constellation, measurements and trends as tiles on the right,\nstations & traffic below the waterfall (amateur radio mode)"); }
            if (ImGui::MenuItem("Drive test")) { _this->applyLayout(2); }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("GPS / coverage map, measurements and trends as tiles on the right"); }
            if (ImGui::MenuItem("Classic (all in the menu)")) { _this->applyLayout(0); }
            ImGui::Separator();
            tw::TextDisabled("Tile content: menu in each tile header. Sizes: drag\nthe dividers. Everything is kept for the next start;\n'Reset' in the tile column restores the preset.");
            ImGui::EndPopup();
        }
        if (_this->decoder_mode != 0) {
            tw::TextColored(tc::warn(), "NETSYMS mode: built-in decoder and decoder-based\nmeasurements are off (see NETSYMS at the bottom).");
        }

        // sections in the user's order (drag the grip on the right of a header,
        // or right-click it)
        for (int id : _this->secOrder) { _this->drawSection(id, menuWidth); }
        _this->applySectionMove();

        // NETSYMS (advanced): own header at the end, closed by default
        std::string nsLabel = std::string(_this->decoder_mode != 0 ? "NETSYMS symbol streaming (active)" : "NETSYMS symbol streaming (advanced)") + "###_tetra_netsyms_hdr_" + _this->name;
        bool nsOpen = ImGui::CollapsingHeader(nsLabel.c_str());
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Streams the demodulated symbols over UDP to an external decoder.\nWhile active, the built-in decoder and all measurements that need it are off.");
        }
        if (nsOpen) {
            if (ImGui::RadioButton(CONCAT("Built-in decoder##_tetra_mode0_", _this->name), _this->decoder_mode == 0) && _this->decoder_mode != 0) {
                _this->decoder_mode = 0; //osmo-tetra
                _this->setMode();
            }
            ImGui::SameLine();
            if (ImGui::RadioButton(CONCAT("NETSYMS over UDP##_tetra_mode1_", _this->name), _this->decoder_mode == 1) && _this->decoder_mode != 1) {
                _this->decoder_mode = 1; //network symbol streaming
                _this->setMode();
            }
        }
        if (nsOpen && _this->decoder_mode != 0) {
            //NETWORK SYM STREAMING
            ImGui::BoxIndicator(ImGui::GetFontSize()*2, _this->symbolExtractor.sync ? IM_COL32(5, 230, 5, 255) : IM_COL32(230, 5, 5, 255));
            tw::SameText(" Sync");
            ImGui::BoxIndicator(menuWidth, _this->tsfound ? IM_COL32(5, 230, 5, 255) : IM_COL32(230, 5, 5, 255));
            tw::SameText(" Training sequences");

            bool netActive = (_this->conn && _this->conn->isOpen());
            if(netActive) { style::beginDisabled(); }
            if (ImGui::InputText(CONCAT("UDP ##_tetrademod_host_", _this->name), _this->hostname, 1023)) {
                config.acquire();
                config.conf[_this->name]["hostname"] = _this->hostname;
                config.release(true);
            }
            ImGui::SameLine();
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::InputInt(CONCAT("##_tetrademod_port_", _this->name), &(_this->port), 0, 0)) {
                config.acquire();
                config.conf[_this->name]["port"] = _this->port;
                config.release(true);
            }
            if(netActive) { style::endDisabled(); }

            if (netActive && ImGui::Button(CONCAT("Net stop##_tetrademod_net_stop_", _this->name), ImVec2(menuWidth, 0))) {
                _this->stopNetwork();
                config.acquire();
                config.conf[_this->name]["sending"] = false;
                config.release(true);
            } else if (!netActive && ImGui::Button(CONCAT("Net start##_tetrademod_net_stop_", _this->name), ImVec2(menuWidth, 0))) {
                _this->startNetwork();
                config.acquire();
                config.conf[_this->name]["sending"] = true;
                config.release(true);
            }

            ImGui::TextUnformatted("Net status:");
            tw::SameNext();
            if (netActive) {
                tw::TextColored(tc::good(), "Sending");
            } else {
                ImGui::TextUnformatted("Idle");
            }
        }
        if(!_this->enabled) {
            style::endDisabled();
        }
    }

    // ---------------- Duplex tables: ETSI (fixed) / amateur radio (file) ----------------

    static std::string rootDir() {
        return (std::string)core::args["root"];
    }

    static std::string hamTablePath() {
        return rootDir() + "/tetra_duplex_ham.json";
    }

    // Default amateur radio values: ETSI, except the 70 cm band (band 4 = 400 MHz)
    // where the duplex spacings commonly used by amateur TETRA repeaters apply.
    static int hamDefault(int d, int b) {
        if (b == 4) {
            switch (d) {
            case 0: return 9000;
            case 1: return 7600;
            case 3: return 8000;
            case 4: return 9400;
            case 5: return 4800;
            case 7: return 7600;
            default: break;
            }
        }
        return tetra_get_duplex_spacing_etsi_khz(d, b);
    }
    static const int HAM_DEFAULTS_VERSION = 2;

    void loadHamTable(const json& legacy) {
        // Start from the amateur radio defaults
        for (int d = 0; d < 8; d++) {
            for (int b = 0; b < 16; b++) { hamTable[d][b] = hamDefault(d, b); }
        }
        std::string path = hamTablePath();
        if (std::filesystem::exists(path)) {
            try {
                std::ifstream f(path);
                json j = json::parse(f, nullptr, true, true);
                for (int d = 0; d < 8; d++) {
                    std::string key = "table_" + std::to_string(d);
                    if (!j.contains(key)) { continue; }
                    for (int b = 0; b < 16 && b < (int)j[key].size(); b++) {
                        int v = j[key][b].get<int>();
                        hamTable[d][b] = std::clamp(v, -1, 1000000);
                    }
                }
                hamTableStatus = "Loaded " + path;
                // Files written before the HAM defaults existed hold the plain ETSI
                // values for band 4: move those (untouched) entries to the defaults.
                int ver = j.contains("defaults_version") ? j["defaults_version"].get<int>() : 1;
                if (ver < HAM_DEFAULTS_VERSION) {
                    for (int d = 0; d < 8; d++) {
                        if (hamTable[d][4] == tetra_get_duplex_spacing_etsi_khz(d, 4)) { hamTable[d][4] = hamDefault(d, 4); }
                    }
                    saveHamTable();
                    hamTableStatus = "Updated to amateur radio defaults (70 cm): " + path;
                }
            }
            catch (const std::exception& e) {
                hamTableStatus = std::string("Error in ") + path + ": " + e.what() + " (using ETSI values, file not overwritten)";
                flog::error("TETRA: {}", hamTableStatus);
                return;
            }
        }
        else {
            if (legacy.is_array()) {
                try {
                    for (int d = 0; d < 8; d++) {
                        for (int b = 0; b < 16; b++) { hamTable[d][b] = legacy[d][b].get<int>(); }
                    }
                }
                catch (...) {}
            }
            saveHamTable();
        }
    }

    void saveHamTable() {
        std::string path = hamTablePath();
        std::string tmp = path + ".new";
        {
            std::ofstream f(tmp);
            if (!f) {
                hamTableStatus = "Cannot write " + path;
                return;
            }
            f << "{\n";
            f << "  \"_comment\": \"TETRA amateur radio duplex spacing tables. table_N = duplex spacing index from SYSINFO, "
                 "16 values = frequency band 0..15 (band n = n*100 MHz). Values in kHz, -1 = not defined. "
                 "Edit here or in SDR++; reload with the button in the TETRA menu.\",\n";
            f << "  \"defaults_version\": " << HAM_DEFAULTS_VERSION << ",\n";
            for (int d = 0; d < 8; d++) {
                f << "  \"table_" << d << "\": [";
                for (int b = 0; b < 16; b++) { f << hamTable[d][b] << (b < 15 ? ", " : ""); }
                f << "]" << (d < 7 ? "," : "") << "\n";
            }
            f << "}\n";
        }
        std::error_code ec;
        std::filesystem::rename(tmp, path, ec);
        hamTableStatus = ec ? ("Cannot write " + path) : ("Saved " + path);
    }

    void applyDuplexMode() {
        if (hamMode) {
            for (int d = 0; d < 8; d++) {
                for (int b = 0; b < 16; b++) { tetra_set_duplex_spacing_khz(d, b, hamTable[d][b]); }
            }
        }
        else {
            tetra_reset_duplex_spacing_etsi();
        }
    }

    void drawDuplexEditor(float menuWidth) {
        if (!sectionHeader(S_DUP, CONCAT("Duplex spacing table##_tetra_dup_", name))) { return; }

        static const char* tableNames = "Table 0\0Table 1\0Table 2\0Table 3\0Table 4\0Table 5\0Table 6\0Table 7\0";
        ImGui::SetNextItemWidth(menuWidth);
        ImGui::Combo(CONCAT("##_tetra_dup_tbl_", name), &duplexEditTable, tableNames);
        if (hamMode) {
            tw::TextDisabled("Amateur radio tables (kHz, -1 = undefined).");
        }
        else {
            tw::TextDisabled("ETSI TS 100 392-15 values (read only).");
        }

        bool changed = false;
        int netTable = osmotetradecoder.getDuplexIdx();
        int netBand = osmotetradecoder.getFreqBand();
        bool locked = (osmotetradecoder.getRxState() == 2);

        if (ImGui::BeginTable(CONCAT("##_tetra_dup_tab_", name), 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV)) {
            ImGui::TableSetupColumn("Band", ImGuiTableColumnFlags_WidthFixed, ImGui::GetFontSize() * 6);
            ImGui::TableSetupColumn("kHz", ImGuiTableColumnFlags_WidthStretch);
            for (int b = 0; b < 16; b++) {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                bool active = locked && netTable == duplexEditTable && netBand == b;
                int32_t etsi = tetra_get_duplex_spacing_etsi_khz(duplexEditTable, b);
                int val = hamMode ? hamTable[duplexEditTable][b] : etsi;
                if (active) {
                    tw::TextColored(tc::good(), "%2d %4d MHz", b, b * 100);
                } else {
                    tw::Text("%2d %4d MHz", b, b * 100);
                }
                ImGui::TableSetColumnIndex(1);
                if (!hamMode) {
                    if (val < 0) { tw::TextDisabled("--"); } else { tw::Text("%d", val); }
                    continue;
                }
                ImGui::SetNextItemWidth(-FLT_MIN);
                bool differs = (val != hamDefault(duplexEditTable, b));
                if (differs) { ImGui::PushStyleColor(ImGuiCol_Text, tc::orange()); }
                if (ImGui::InputInt(CONCAT("##_tetra_dup_v_" + std::to_string(duplexEditTable) + "_" + std::to_string(b) + "_", name), &val, 0, 0)) {
                    hamTable[duplexEditTable][b] = std::clamp(val, -1, 1000000);
                    changed = true;
                }
                if (differs) { ImGui::PopStyleColor(); }
            }
            ImGui::EndTable();
        }

        if (hamMode) {
            float w3 = menuWidth / 3.0f - 4;
            if (ImGui::Button(CONCAT("Table = default##_tetra_dup_rst1_", name), ImVec2(w3, 0))) {
                for (int b = 0; b < 16; b++) { hamTable[duplexEditTable][b] = hamDefault(duplexEditTable, b); }
                changed = true;
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Amateur radio defaults: ETSI values, 70 cm (band 4):\n"
                                  "Table 0 9.0 MHz, 1 7.6 MHz, 3 8.0 MHz, 4 9.4 MHz, 5 4.8 MHz, 7 7.6 MHz");
            }
            if (tw::SameButton(CONCAT("Reload file##_tetra_dup_rl_", name), ImVec2(w3, 0))) {
                loadHamTable(json());
                applyDuplexMode();
            }
            if (tw::SameButton(CONCAT("Save file##_tetra_dup_sv_", name), ImVec2(w3, 0))) {
                saveHamTable();
            }
            if (changed) {
                saveHamTable();
                applyDuplexMode();
            }
            tw::TextDisabled("%s", hamTableStatus.c_str());
        }
        tw::TextDisabled("Changes apply with the next SYSINFO.");
    }

    // ---------------- AFC (VFO follows the carrier) and automatic RF gain ----------------

    SourceGainInterface* findGainInterface() {
        std::string sel = sigpath::sourceManager.getSelectedName();
        auto list = core::moduleManager.getAllInterfaces<SourceGainInterface>("SourceGainInterface");
        for (auto* g : list) {
            if (g && g->gainSourceName() == sel) { return g; }
        }
        return nullptr;
    }

    SourceFreqCorrInterface* findFreqInterface() {
        std::string sel = sigpath::sourceManager.getSelectedName();
        auto list = core::moduleManager.getAllInterfaces<SourceFreqCorrInterface>("SourceFreqCorrInterface");
        for (auto* f : list) {
            if (f && f->corrSourceName() == sel) { return f; }
        }
        return nullptr;
    }

    void drawAfcGain(float menuWidth, const TetraMeasSnapshot& m) {
        ImVec4 good = tc::good();
        ImVec4 warn = tc::warn();
        if (ImGui::Checkbox(CONCAT("AFC (follow carrier)##_tetra_afc_", name), &afcEnabled)) {
            config.acquire();
            config.conf[name]["afc"] = afcEnabled;
            config.release(true);
            afcAnchorValid = false;
            afcStatus.clear();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Moves the VFO so the carrier stays centered (for SDRs that drift).\n"
                              "Only acts while the decoder is locked; locks the cell automatically.\n"
                              "Pull-in limited to +-6 kHz around the frequency the cell announces\n(SYSINFO; otherwise from where AFC started). The value is the offset from it.\n"
                              "Leave off with a GPS/TCXO-stabilised SDR for pure measurements.");
        }
        if (afcEnabled) {
            tw::SameNext();
            if (afcAnchorValid) {
                tw::TextColored(afcStatus.empty() ? good : warn, "%+.0f Hz", afcLastSet - afcAnchor);
            }
            else {
                tw::TextDisabled("waiting for lock");
            }
            if (!afcStatus.empty()) { tw::SameTextColored(warn, "(limit)"); if (ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", afcStatus.c_str()); } }
        }

        drawSdrAdvice(menuWidth);
        if (ImGui::Checkbox(CONCAT("Lock VFO##_tetra_vfolock_", name), &vfoLocked)) {
            config.acquire();
            config.conf[name]["vfoLocked"] = vfoLocked;
            config.release(true);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("The mouse no longer tunes the TETRA VFO: clicking or dragging in the\n"
                              "spectrum / waterfall moves the view instead, the wheel does nothing.\n"
                              "When the view is dragged past the band edge the SDR is retuned and the\n"
                              "VFO keeps its frequency (the view stops where the VFO would leave the band).\n"
                              "AFC and frequency entry still work.");
        }
        {
            std::string sl = std::string("Snap to carrier##_tetra_snap_") + name;
            if (tw::SameCheckbox(sl.c_str(), &snapCarrier)) {
                config.acquire();
                config.conf[name]["snapCarrier"] = snapCarrier;
                config.release(true);
            }
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("A click / drag with the mouse that tunes the TETRA VFO ends on the centre of\n"
                              "the strongest carrier within +-25 kHz (spectrum of the raw samples, 6.25 kHz\n"
                              "raster when within 1 kHz of it). Works zoomed out as well. Then tick Lock VFO\n"
                              "to keep it there while moving the view.");
        }
        if (!snapStatus.empty()) { tw::SameTextDisabled("%s", snapStatus.c_str()); }

        SourceGainInterface* g = findGainInterface();
        if (ImGui::Checkbox(CONCAT("Auto RF gain##_tetra_agc_", name), &autoGain)) {
            config.acquire();
            config.conf[name]["autoGain"] = autoGain;
            config.release(true);
            gainStatus.clear();
            gainLastSet = -1;
            restartLevelling();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Looks for the best signal-to-noise ratio with as little gain as possible:\n"
                              "steps up (3 dB every 4 s) while the SNR still improves, steps back once\n"
                              "the noise floor only rises with the gain, and goes down at once when the\n"
                              "ADC peak over the whole SDR bandwidth or the TETRA peak nears full scale.\n"
                              "Supported: RTL-SDR (tuner AGC off), PlutoSDR (gain mode Manual),\n"
                              "Airspy (Sensitive or Linear mode), USRP, HackRF (LNA + VGA),\n"
                              "BladeRF (gain mode Manual), LimeSDR, SDRplay (IF gain, IF AGC off).\n"
                              "Spectran: set the reference level in the Spectran software.\n"
                              "Strong signals elsewhere in the SDR bandwidth are taken into account\n"
                              "(ADC peak of the whole band).");
        }
        tw::SameNext();
        if (g) {
            if (g->gainControllable()) {
                tw::TextColored(autoGain ? good : tc::grey(), "%s", g->gainLabel(g->gainStep()).c_str());
            }
            else {
                tw::TextDisabled("(hardware AGC on / not running)");
            }
        }
        else {
            // Advisor for sources without gain interface
            if (m.valid) {
                if (m.peakDbfs > -6.0f || m.levelDbfs > -20.0f) { tw::TextColored(warn, "reduce gain"); }
                else if (m.levelDbfs < -35.0f) { tw::TextColored(warn, "increase gain"); }
                else { tw::TextColored(good, "gain OK"); }
            }
        }
        if (!gainStatus.empty()) { tw::SameTextDisabled("(?)"); if (ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", gainStatus.c_str()); } }
        if (autoGain && g && g->gainControllable()) {
            ImGui::SetNextItemWidth((std::min)(menuWidth * 0.55f, 170.0f * style::uiScale));
            if (ImGui::Combo(CONCAT("##_tetra_gmode_", name), &gainMode, "Level and hold\0Follow\0Automatic (GPS)\0")) {
                config.acquire();
                config.conf[name]["gainMode"] = gainMode;
                config.release(true);
                restartLevelling();
            }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Level and hold: find the best gain once, then keep it fixed, so all\n"
                                  "results of a measurement series are taken with the same receiver\n"
                                  "settings. A warning asks for re-levelling if the ADC nears clipping\n"
                                  "or the SNR drops by more than 6 dB.\n"
                                  "Follow: adjust continuously (every step is logged).\n"
                                  "Automatic: follow while moving (GPS speed above 5 km/h over 10 s),\n"
                                  "hold after 30 s standing still or without GPS fix. With a level\n"
                                  "calibration (dBm) it always holds - the calibration is only valid\n"
                                  "at the gain it was made with.");
            }
            if (tw::SameButtonOrFull(CONCAT("Re-level##_tetra_relevel_", name))) {
                restartLevelling();
                meas.addEvent("Re-levelling requested");
            }
            bool follow = gainFollowNow();
            if (follow) { tw::TextColored(good, "following"); if (gainMoving) { tw::SameTextDisabled("(moving, %.0f km/h)", gainSpeedAvg); } }
            else if (gainHolding) { tw::TextColored(good, "held"); tw::SameTextDisabled("(levelled %s)", gainHeldAt.c_str()); }
            else { tw::TextColored(warn, "levelling..."); }
            if (!gainHoldWarn.empty()) { tw::TextColored(tc::bad(), "%s", gainHoldWarn.c_str()); }
            if (!calAllowsFollow() && gainMode == 1) { tw::TextColored(warn, "Level calibration without known gain: dBm wrong after gain steps."); }
            if (!calAllowsFollow() && gainMode == 2 && gainMoving) { tw::TextDisabled("moving, but held: calibration gain unknown"); }
        }
        // whole-band view: ADC peak over the full SDR bandwidth and channel SNR
        tw::Text("Band:"); tw::SameNext();
        if (bandRes.valid) {
            ImVec4 pc = bandRes.peakDbfs > -3.0f ? tc::bad() : (bandRes.peakDbfs > -10.0f ? tc::warn() : tc::good());
            tw::TextColored(pc, "ADC peak %.1f dBFS", bandRes.peakDbfs);
            if (!std::isnan(bandRes.snrDb)) { tw::SameText(" SNR"); tw::SameTextColored(tc::val(), "%.1f dB", bandRes.snrDb); }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("ADC peak: largest I/Q value over the whole SDR bandwidth in the last second\n"
                                  "(any signal, not only TETRA). Above -3 dBFS the ADC clips.\n"
                                  "SNR: TETRA channel against the noise floor 30-400 kHz around it\n"
                                  "(per 1 kHz bin, coarse). Auto RF gain looks for the lowest gain\n"
                                  "that still gives the best SNR.");
            }
        }
        else { tw::TextDisabled("--"); }
    }

    // Called every frame from the GUI thread
    // Snap to carrier: watch mouse tunes of the TETRA VFO (GUI thread)
    void snapLoop() {
        if (!vfo || !vfo->wtfVFO) { snapPending = false; snapMouseDown = false; return; }
        double cur = gui::waterfall.getCenterFrequency() + vfo->getOffset();
        bool inSpec = gui::waterfall.mouseInFFT || gui::waterfall.mouseInWaterfall;
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && inSpec) {
            snapMouseDown = true;
            snapStartFreq = cur;
        }
        if (snapMouseDown && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            snapMouseDown = false;
            bool ours = gui::waterfall.selectedVFO == name;
            if (snapCarrier && !vfoLocked && ours && std::fabs(cur - snapStartFreq) > 1.0) {
                snapPending = true;
                snapClickOffset = vfo->getOffset();
                snapReqTime = std::chrono::steady_clock::now();
                snapStatus = "searching...";
            }
        }
        if (!snapPending) { return; }
        // a newer tune (frequency entry, another click) cancels the snap
        if (std::fabs(vfo->getOffset() - snapClickOffset) > 1.0) { snapPending = false; snapStatus.clear(); return; }
        double off = 0.0;
        int r = bandMon.snapCarrier(snapClickOffset, off);
        if (r == 1) {
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - snapReqTime).count() > 4.0) {
                snapPending = false;
                snapStatus = "no samples";
            }
            return;
        }
        snapPending = false;
        if (r == 2) { snapStatus = "no carrier near the click"; return; }
        snapStatus.clear();
        tuner::tune(tuner::TUNER_MODE_NORMAL, name, gui::waterfall.getCenterFrequency() + off);
    }

    void controlLoops() {
        if (vfo && vfo->wtfVFO) { vfo->wtfVFO->positionLocked = enabled && vfoLocked; }
        updatePanels();
        if (!enabled) { return; }
        snapLoop();
        double propT = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
        if (gps.isRunning() && propView.due(propT)) {
            // propagation diagram: level / MER with position (only the locked cell, if locked)
            GpsFix pf = gps.get();
            TetraMeasSnapshot pm = meas.get();
            bool cellOk = !meas.getCellLock().active || pm.cellMatch == 1;
            float lv = (pm.valid && cellOk) ? pm.levelDbfs + levelCalEff : NAN;
            float mr = (pm.valid && cellOk && pm.symSync) ? pm.merDb : NAN;
            propView.feed(propT, pf.valid, pf.lat, pf.lon, lv, mr);
        }
        {
            // dBm calibration follows the RF gain (difference from the gain table)
            float eff = levelCalDb;
            SourceGainInterface* cg = findGainInterface();
            if (levelCalDb != 0.0f && !std::isnan(levelCalGainDb) && cg && cg->gainControllable()) {
                eff = levelCalDb - ((float)cg->gainStepDb(cg->gainStep()) - levelCalGainDb);
            }
            if (eff != levelCalEff) { levelCalEff = eff; meas.setLevelCal(levelCalEff); }
        }
        // first start (centre still at the edition default 439 MHz): put the
        // top of the SDR band at 440 MHz, where the amateur TETRA cells are
        if (!startViewDone && gui::mainWindow.sdrIsRunning() && sigpath::iqFrontEnd.getSampleRate() > 0) {
            startViewDone = true;
            if (std::fabs(gui::waterfall.getCenterFrequency() - 439000000.0) < 1.0) { applyStartView(); }
            config.acquire();
            config.conf[name]["startViewV21"] = true;
            config.release(true);
        }
        auto now = std::chrono::steady_clock::now();
        if (bandMon.isRunning() && std::chrono::duration<double>(now - bandLast).count() >= 1.0) {
            bandLast = now;
            bandRes = bandMon.update(vfo ? vfo->getOffset() : 0.0);
        }
        {
            // lock the source's gain controls while auto RF gain steps them;
            // "Release" in the source menu switches auto RF gain off
            SourceGainInterface* gi = findGainInterface();
            if (gi) {
                if (gi->gainReleaseReq.exchange(false)) {
                    autoGain = false;
                    gainLastSet = -1;
                    meas.addEvent("Auto RF gain off (released in the source menu)");
                    config.acquire();
                    config.conf[name]["autoGain"] = autoGain;
                    config.release(true);
                }
                gi->gainLockedExt.store(autoGain && gi->gainControllable());
                // lost samples reported by the SDR (Pluto: ADC core overflow)
                long long ov = gi->overflowCount();
                if (gi != ovIf) { ovIf = gi; ovLast = ov; }
                else if (ov >= 0 && ovLast >= 0 && ov > ovLast) {
                    meas.addEvent("SDR lost samples: " + std::to_string(ov - ovLast) + " overflow(s), " + std::to_string(ov) + " since start");
                    lossThisSecond = true;
                }
                ovLast = ov;
            }
        }
        TetraMeasSnapshot m = meas.get();
        if (!m.valid) { return; }

        // ---- AFC ----
        if (afcEnabled && vfo && decoder_mode == 0) {
            // only act on a trustworthy estimate: decoder locked, symbol sync and
            // a steady burst flow (an unstable lock can report wrong offsets)
            bool locked = (m.rxState == 2) && m.symSync && m.bursts >= 30;
            double cur = vfo->getOffset();
            if (afcAnchorValid && std::fabs(cur - afcLastSet) > 1.0) {
                // user retuned: start over from the new position
                afcAnchorValid = false;
                afcStatus.clear();
            }
            if (locked && std::chrono::duration<double>(now - afcLastStep).count() >= 3.0 && afcStatus.empty()) {
                if (!afcAnchorValid) {
                    afcAnchor = cur;
                    // anchor on the frequency the cell announces (SYSINFO), so the
                    // display shows the real carrier error and not where the VFO
                    // happened to be placed
                    double vf = mainVfoFreq();
                    if (m.dlFreq > 1e6 && vf > 1e6 && std::fabs((double)m.dlFreq - vf) < 8000.0) {
                        afcAnchor = cur + ((double)m.dlFreq - vf);
                    }
                    afcLastSet = cur;
                    afcAnchorValid = true;
                    // AFC follows exactly this carrier: lock the cell as well
                    TetraMeasurements::CellLock lock = meas.getCellLock();
                    if (!lock.active && m.mcc != 0) {
                        lock.active = true;
                        lock.mcc = m.mcc; lock.mnc = m.mnc; lock.cc = m.cc; lock.dlFreq = m.dlFreq;
                        meas.setCellLock(lock);
                    }
                }
                double err = m.carrierOffsetHz;
                if (std::fabs(err) > 15.0) {
                    double step = std::clamp(0.6 * err, -2000.0, 2000.0);
                    double target = cur + step;
                    if (std::fabs(target - afcAnchor) > 6000.0) {
                        afcStatus = "AFC limit (+-6 kHz) reached - check tuning";
                    }
                    else {
                        sigpath::vfoManager.setOffset(name, target);
                        meas.addEvent("AFC step " + std::to_string((int)std::lround(step)) + " Hz");
                        afcLastSet = target;
                    }
                }
                afcLastStep = now;
            }
        }

        // ---- movement (automatic gain mode): GPS speed averaged over ~10 s ----
        if (std::chrono::duration<double>(now - gainMoveLast).count() >= 1.0) {
            gainMoveLast = now;
            GpsFix f = gps.isRunning() ? gps.get() : GpsFix();
            bool ok = f.valid && f.ageS < 5.0;
            gainSpeedAvg = ok ? gainSpeedAvg + (f.speedKmh - gainSpeedAvg) / 10.0 : 0.0;
            if (ok && gainSpeedAvg > 5.0) {
                gainStillS = 0;
                if (!gainMoving) {
                    gainMoving = true;
                    if (autoGain && gainMode == 2 && calAllowsFollow()) { meas.addEvent("Moving: auto RF gain follows the level"); gainHoldWarn.clear(); }
                }
            }
            else if (gainMoving && ++gainStillS >= 30) {
                gainMoving = false;
                if (autoGain && gainMode == 2 && calAllowsFollow()) {
                    // standing: keep the gain found while moving
                    fixGain("standing still");
                }
            }
        }

        // ---- automatic RF gain ----
        if (autoGain && std::chrono::duration<double>(now - gainLastStep).count() >= 4.0) {
            gainLastStep = now;
            SourceGainInterface* g = findGainInterface();
            if (!g) {
                gainStatus = "Source has no gain control interface (supported: RTL-SDR, PlutoSDR, Airspy, USRP, HackRF, BladeRF, LimeSDR, SDRplay)";
            }
            else if (g->gainControllable()) {
                gainStatus.clear();
                int step = g->gainStep();
                int n = g->gainStepCount();
                if (g != gainLastIf) { gainLastIf = g; gainLastSet = -1; }   // other source
                // gain changed by hand in the source menu: the user takes over
                if (gainLastSet >= 0 && step != gainLastSet) {
                    autoGain = false;
                    gainLastSet = -1;
                    meas.addEvent("Auto RF gain off: gain changed by hand (" + g->gainLabel(step) + ")");
                    config.acquire();
                    config.conf[name]["autoGain"] = autoGain;
                    config.release(true);
                    return;
                }
                gainLastSet = step;
                double cur = g->gainStepDb(step);
                double want = cur;
                bool follow = gainFollowNow();
                if (!follow && gainHolding) {
                    // held: no steps, only advise re-levelling
                    TetraBandMonitor::Result hb = bandRes;
                    double pk = hb.valid ? (double)hb.peakDbfs : -200.0;
                    std::string w;
                    if (pk > -3.0 || m.peakDbfs > -3.0f) { w = "ADC near clipping - press Re-level"; }
                    else if (hb.valid && !std::isnan(hb.snrDb) && !std::isnan(gainHoldSnr) && hb.snrDb < gainHoldSnr - 6.0f) {
                        w = "SNR more than 6 dB below the levelled value - re-level?";
                    }
                    if (!w.empty() && w != gainHoldWarn) { meas.addEvent("Gain held: " + w); }
                    gainHoldWarn = w;
                    return;
                }
                std::string why;
                TetraBandMonitor::Result b = bandRes;
                bool haveSnr = b.valid && !std::isnan(b.snrDb);
                double bandPk = b.valid ? (double)b.peakDbfs : -200.0;
                if (!std::isnan(agcCeilDb) && std::chrono::duration<double>(now - agcCeilT).count() > 120.0) { agcCeilDb = NAN; }
                if (bandPk > -3.0 || m.peakDbfs > -3.0f || m.levelDbfs > -12.0f) {
                    // overload, in the channel or anywhere in the SDR bandwidth: down fast
                    want = cur - std::clamp((std::max)({ bandPk + 12.0, (double)m.peakDbfs + 12.0, (double)m.levelDbfs + 30.0 }), 3.0, 10.0);
                    why = bandPk > -3.0 && m.peakDbfs <= -3.0f ? " (ADC overload by a signal in the band)" : " (overload)";
                    agcCeilDb = want;
                    agcCeilT = now;
                }
                else if (!haveSnr) {
                    // no spectrum: level window -35 ... -20 dBFS, peak below -10 dBFS
                    if (m.peakDbfs > -6.0f || m.levelDbfs > -20.0f) {
                        want = cur - std::clamp((std::max)((double)m.peakDbfs + 10.0, (double)m.levelDbfs + 27.0), 2.0, 10.0);
                    }
                    else if (m.levelDbfs < -35.0f && m.peakDbfs < -14.0f) {
                        want = cur + std::clamp((std::min)(-27.0 - (double)m.levelDbfs, -10.0 - (double)m.peakDbfs), 2.0, 10.0);
                    }
                }
                else {
                    // best signal-to-noise with as little gain as possible: go up while
                    // the SNR still improves (the SDR's own noise dominates); once the
                    // noise floor rises with the gain, more gain only costs headroom
                    if (agcLastUp && !std::isnan(agcPrevSnr) && b.snrDb - agcPrevSnr < 1.0) {
                        want = agcPrevGainDb;
                        why = " (no SNR gain above this)";
                        agcCeilDb = agcPrevGainDb;
                        agcCeilT = now;
                    }
                    else if (b.snrDb < 35.0f && bandPk < -10.0 && m.peakDbfs < -10.0f && (std::isnan(agcCeilDb) || cur + 2.5 <= agcCeilDb)) {
                        want = cur + 3.0;
                    }
                    else if (b.snrDb > 45.0f && bandPk > -20.0) {
                        want = cur - 3.0;
                        why = " (SNR ample, more headroom)";
                    }
                }
                agcLastUp = false;
                if (want != cur) {
                    int best = step;
                    double bestErr = 1e9;
                    for (int s = 0; s < n; s++) {
                        double e = std::fabs(g->gainStepDb(s) - want);
                        if (e < bestErr) { bestErr = e; best = s; }
                    }
                    if (best == step) { best = std::clamp(step + (want > cur ? 1 : -1), 0, n - 1); }
                    if (best != step) {
                        g->setGainStep(best);
                        gainLastSet = g->gainStep();
                        agcLastUp = g->gainStepDb(best) > cur;
                        agcPrevSnr = haveSnr ? (double)b.snrDb : NAN;
                        agcPrevGainDb = cur;
                        meas.addEvent(std::string(want > cur ? "RF gain up to " : "RF gain down to ") + g->gainLabel(best) + why);
                        gainQuiet = 0;
                        return;
                    }
                }
                // no step this time: levelling is done after two quiet evaluations
                if (!follow && ++gainQuiet >= 2) { fixGain("levelled"); }
            }
        }
    }

    // a level calibration (dBm) is only valid at the gain it was made with:
    // automatic mode then holds even while moving
    // centre = 440 MHz - 0.45 x sample rate: the band ends just above 440 MHz
    void applyStartView() {
        double sr = sigpath::iqFrontEnd.getSampleRate();
        if (!(sr > 0)) { return; }
        double f = std::round((440000000.0 - 0.45 * sr) / 1000.0) * 1000.0;
        gui::waterfall.setCenterFrequency(f);
        gui::waterfall.centerFreqMoved = true;
        meas.addEvent("Start view: centre " + std::to_string((long long)std::llround(f / 1000.0)) + " kHz (band up to 440 MHz)");
    }

    // recommended sample rates of the common SDRs (TETRA downlink; duplex needs more)
    struct SdrAdvice { const char* match; double minSr, maxSr; const char* note; };
    static const SdrAdvice* sdrAdvice(const std::string& src) {
        static const SdrAdvice tbl[] = {
            { "PlutoSDR", 3e6, 7e6, "3-5 MS/s (USB 2.0 up to about 7 MS/s), gain mode Manual" },
            { "RTL", 1e6, 2.4e6, "up to 2.4 MS/s, tuner AGC off" },
            { "USRP", 4e6, 16e6, "4 MS/s for the downlink; 12 MS/s or more for downlink + uplink at 9.4 MHz duplex" },
            { "Airspy", 2.5e6, 10e6, "2.5-10 MS/s, Linear or Sensitive mode" },
            { "SDRplay", 2e6, 10e6, "2-8 MS/s, IF AGC off" },
            { "HackRF", 2e6, 20e6, "2-10 MS/s (8 bit: little dynamic range)" },
            { "BladeRF", 2e6, 20e6, "2-10 MS/s, gain mode Manual" },
            { "LimeSDR", 2e6, 20e6, "2-10 MS/s" },
        };
        for (auto& a : tbl) { if (src.find(a.match) != std::string::npos) { return &a; } }
        return nullptr;
    }
    void drawSdrAdvice(float menuWidth) {
        const std::string& src = sigpath::sourceManager.getSelectedName();
        double sr = sigpath::iqFrontEnd.getSampleRate();
        tw::Text("SDR:"); tw::SameNext();
        const SdrAdvice* a = sdrAdvice(src);
        bool ok = !a || (sr >= a->minSr - 1.0 && sr <= a->maxSr + 1.0);
        tw::TextColored(ok ? tc::val() : tc::warn(), "%s %.2f MS/s", src.c_str(), sr / 1e6);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Recommended: %s", a ? a->note : "a few MS/s around the TETRA channel");
        }
        if (!ok) { tw::SameTextColored(tc::warn(), "(see tooltip)"); }
        if (tw::SameSmallButton(CONCAT("Recommended view##_tetra_startview_", name))) { applyStartView(); }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Centre the SDR so that its band ends just above 440 MHz\n(centre = 440 MHz - 0.45 x sample rate)."); }
        (void)menuWidth;
    }

    bool gainFollowNow() const { return gainMode == 1 || (gainMode == 2 && gainMoving && calAllowsFollow()); }
    // dBm stays right while following only if the gain at calibration is known
    bool calAllowsFollow() const { return levelCalDb == 0.0f || !std::isnan(levelCalGainDb); }

    void restartLevelling() {
        gainHolding = false;
        gainQuiet = 0;
        gainHoldWarn.clear();
        agcCeilDb = NAN;
    }

    // keep the current gain from now on (level and hold)
    void fixGain(const char* why) {
        gainHolding = true;
        gainQuiet = 0;
        gainHoldWarn.clear();
        gainHoldSnr = (bandRes.valid && !std::isnan(bandRes.snrDb)) ? bandRes.snrDb : NAN;
        time_t t = time(nullptr);
        char b[16];
        struct tm lt;
#ifdef _WIN32
        localtime_s(&lt, &t);
#else
        localtime_r(&t, &lt);
#endif
        strftime(b, sizeof(b), "%H:%M:%S", &lt);
        gainHeldAt = b;
        SourceGainInterface* g = findGainInterface();
        std::string lbl = (g && g->gainControllable()) ? g->gainLabel(g->gainStep()) : std::string("?");
        meas.addEvent(std::string("RF gain held at ") + lbl + " (" + why + ")");
    }


    // ---------------- Menu sections in a user-defined order ----------------

    enum SectionId { S_MEAS, S_TXA, S_TREND, S_CARR, S_BURST, S_MP, S_ALARM, S_REPORT, S_GPS, S_NET, S_SUBS, S_DUP, S_CONST, S_COUNT };
    static const char* sectionKey(int id) {
        static const char* k[S_COUNT] = { "meas", "txa", "trend", "carriers", "bursts", "multipath", "alarms", "report", "gps", "network", "stations", "duplex", "constellation" };
        return (id >= 0 && id < S_COUNT) ? k[id] : "";
    }
    void defaultSectionOrder() {
        secOrder.clear();
        secOrder.push_back(S_CONST);   // constellation first (v0.21: own section)
        for (int i = 0; i < S_COUNT; i++) { if (i != S_CONST) { secOrder.push_back(i); } }
    }
    void loadSectionOrder() {
        defaultSectionOrder();
        if (!config.conf[name].contains("sectionOrder") || !config.conf[name]["sectionOrder"].is_array()) { return; }
        std::vector<int> o;
        for (auto& v : config.conf[name]["sectionOrder"]) {
            if (!v.is_string()) { continue; }
            std::string k = v.get<std::string>();
            for (int i = 0; i < S_COUNT; i++) {
                if (k == sectionKey(i) && std::find(o.begin(), o.end(), i) == o.end()) { o.push_back(i); }
            }
        }
        // new sections: the constellation goes above the measurements, others to the end
        if (std::find(o.begin(), o.end(), (int)S_CONST) == o.end()) {
            auto it = std::find(o.begin(), o.end(), (int)S_MEAS);
            o.insert(it, (int)S_CONST);
        }
        for (int i = 0; i < S_COUNT; i++) { if (std::find(o.begin(), o.end(), i) == o.end()) { o.push_back(i); } }
        secOrder = o;
    }
    void saveSectionOrder() {
        config.acquire();
        config.conf[name]["sectionOrder"] = json::array();
        for (int id : secOrder) { config.conf[name]["sectionOrder"].push_back(sectionKey(id)); }
        config.release(true);
    }
    // move 'src' to the position of 'dst' (before or after it); applied after the frame's loop
    void requestSectionMove(int src, int dst, bool after) { secMoveSrc = src; secMoveDst = dst; secMoveAfter = after; }
    void applySectionMove() {
        if (secResetReq) { secResetReq = false; defaultSectionOrder(); saveSectionOrder(); }
        if (secMoveSrc < 0) { return; }
        int src = secMoveSrc, dst = secMoveDst;
        bool after = secMoveAfter;
        secMoveSrc = -1;
        if (src == dst) { return; }
        auto it = std::find(secOrder.begin(), secOrder.end(), src);
        if (it == secOrder.end()) { return; }
        secOrder.erase(it);
        if (dst == -1) { secOrder.insert(secOrder.begin(), src); }          // top
        else if (dst == -2) { secOrder.push_back(src); }                     // bottom
        else {
            auto d = std::find(secOrder.begin(), secOrder.end(), dst);
            if (d == secOrder.end()) { secOrder.push_back(src); }
            else { secOrder.insert(after ? d + 1 : d, src); }
        }
        saveSectionOrder();
    }
    bool sectionVisible(int id) {
        if (id == S_NET || id == S_DUP) { return decoder_mode == 0; }
        if (id == S_SUBS) { return decoder_mode == 0 && hamMode; }
        return true;
    }

    // Collapsing header with a drag grip on the right, drop target and a
    // context menu (move up / down / top / bottom / reset)
    bool sectionHeader(int id, const char* label, ImGuiTreeNodeFlags flags = 0) {
        bool open = ImGui::CollapsingHeader(label, flags | ImGuiTreeNodeFlags_AllowItemOverlap);
        ImVec2 hMin = ImGui::GetItemRectMin(), hMax = ImGui::GetItemRectMax();
        // drop target: the header; the line shows where the section lands
        if (ImGui::BeginDragDropTarget()) {
            if (const ImGuiPayload* pl = ImGui::AcceptDragDropPayload("TETRA_SECTION", ImGuiDragDropFlags_AcceptBeforeDelivery | ImGuiDragDropFlags_AcceptNoDrawDefaultRect)) {
                int src = *(const int*)pl->Data;
                bool after = ImGui::GetMousePos().y > (hMin.y + hMax.y) * 0.5f;
                if (src != id) {
                    float y = after ? hMax.y + 1.0f : hMin.y - 2.0f;
                    ImU32 c = ImGui::GetColorU32(ImGuiCol_DragDropTarget);
                    ImDrawList* fdl = ImGui::GetForegroundDrawList();
                    fdl->AddLine(ImVec2(hMin.x, y), ImVec2(hMax.x, y), c, 3.0f);
                    fdl->AddCircleFilled(ImVec2(hMin.x, y), 4.0f, c);
                    fdl->AddCircleFilled(ImVec2(hMax.x, y), 4.0f, c);
                    if (pl->IsDelivery()) { requestSectionMove(src, id, after); }
                }
            }
            ImGui::EndDragDropTarget();
        }
        if (ImGui::BeginPopupContextItem((std::string("##_tetra_secctx_") + sectionKey(id) + name).c_str())) {
            auto pos = std::find(secOrder.begin(), secOrder.end(), id) - secOrder.begin();
            // neighbours among the sections that are shown
            int prev = -1, next = -1;
            for (int i = (int)pos - 1; i >= 0; i--) { if (sectionVisible(secOrder[i])) { prev = secOrder[i]; break; } }
            for (int i = (int)pos + 1; i < (int)secOrder.size(); i++) { if (sectionVisible(secOrder[i])) { next = secOrder[i]; break; } }
            if (ImGui::MenuItem("Move up", nullptr, false, prev >= 0)) { requestSectionMove(id, prev, false); }
            if (ImGui::MenuItem("Move down", nullptr, false, next >= 0)) { requestSectionMove(id, next, true); }
            if (ImGui::MenuItem("Move to top", nullptr, false, prev >= 0)) { requestSectionMove(id, -1, false); }
            if (ImGui::MenuItem("Move to bottom", nullptr, false, next >= 0)) { requestSectionMove(id, -2, true); }
            ImGui::Separator();
            if (ImGui::MenuItem("Reset order")) { secResetReq = true; }
            ImGui::EndPopup();
        }
        // grip on the right end of the header: the drag source
        float gw = ImGui::GetFontSize() * 1.2f;
        ImVec2 back = ImGui::GetCursorScreenPos();
        ImGui::SetCursorScreenPos(ImVec2(hMax.x - gw - ImGui::GetStyle().FramePadding.x, hMin.y));
        ImGui::InvisibleButton((std::string("##_tetra_secgrip_") + sectionKey(id) + name).c_str(), ImVec2(gw, hMax.y - hMin.y));
        bool gh = ImGui::IsItemHovered() || ImGui::IsItemActive();
        if (gh) { ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll); }
        {
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 gmin = ImGui::GetItemRectMin(), gmax = ImGui::GetItemRectMax();
            ImU32 dc = ImGui::GetColorU32(gh ? ImGuiCol_Text : ImGuiCol_TextDisabled);
            float r = (std::max)(1.2f, ImGui::GetFontSize() * 0.09f);
            float cx = (gmin.x + gmax.x) * 0.5f, cy = (gmin.y + gmax.y) * 0.5f, dx = r * 2.6f, dy = r * 2.6f;
            for (int i = -1; i <= 1; i++) {
                dl->AddCircleFilled(ImVec2(cx - dx * 0.5f, cy + i * dy), r, dc);
                dl->AddCircleFilled(ImVec2(cx + dx * 0.5f, cy + i * dy), r, dc);
            }
        }
        if (gh && !ImGui::IsItemActive()) { ImGui::SetTooltip("Drag to move this section, right-click the header for more"); }
        // own window: icon left of the grip (blue while the window is open)
        if (bool* df = detachFlag(id)) {
            float iw = ImGui::GetFontSize() * 1.25f;
            ImVec2 keep = ImGui::GetCursorScreenPos();
            ImGui::SetCursorScreenPos(ImVec2(hMax.x - gw - iw - 2.0f * ImGui::GetStyle().FramePadding.x, hMin.y));
            if (ImGui::InvisibleButton((std::string("##_tetra_secwin_") + sectionKey(id) + name).c_str(), ImVec2(iw, hMax.y - hMin.y))) { setDetached(id, !*df); }
            bool wh = ImGui::IsItemHovered();
            ImVec2 bmin = ImGui::GetItemRectMin(), bmax = ImGui::GetItemRectMax();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImU32 c = *df ? ImGui::GetColorU32(ImGuiCol_CheckMark) : ImGui::GetColorU32(wh ? ImGuiCol_Text : ImGuiCol_TextDisabled);
            float sz = ImGui::GetFontSize() * 0.62f, th = (std::max)(1.0f, ImGui::GetFontSize() * 0.08f);
            float cx = (bmin.x + bmax.x) * 0.5f, cy = (bmin.y + bmax.y) * 0.5f;
            ImVec2 a(cx - sz * 0.5f, cy - sz * 0.35f), b(cx + sz * 0.3f, cy + sz * 0.5f);
            dl->AddRect(a, b, c, 0.0f, 0, th);
            ImVec2 t0(cx - sz * 0.05f, cy + sz * 0.05f), t1(cx + sz * 0.55f, cy - sz * 0.55f);
            dl->AddLine(t0, t1, c, th);
            dl->AddLine(t1, ImVec2(t1.x - sz * 0.35f, t1.y), c, th);
            dl->AddLine(t1, ImVec2(t1.x, t1.y + sz * 0.35f), c, th);
            if (wh) { ImGui::SetTooltip(*df ? "Own window is open - click to put it back into the menu" : "Show this section in its own window"); }
            ImGui::SetCursorScreenPos(keep);
        }
        if (ImGui::BeginDragDropSource(ImGuiDragDropFlags_SourceNoHoldToOpenOthers)) {
            ImGui::SetDragDropPayload("TETRA_SECTION", &id, sizeof(int));
            const char* hash = strstr(label, "##");
            ImGui::TextUnformatted(label, hash ? hash : nullptr);
            ImGui::EndDragDropSource();
        }
        ImGui::SetCursorScreenPos(back);
        return open;
    }

    void drawSection(int id, float menuWidth) {
        if (!sectionVisible(id)) { return; }
        if (inTiles(id)) {
            ImGuiTreeNodeFlags fl = (id == S_MEAS || id == S_CONST || id == S_NET || id == S_SUBS) ? ImGuiTreeNodeFlags_DefaultOpen : 0;
            if (sectionHeader(id, (sectionLabel(id) + name).c_str(), fl)) { tw::TextDisabled("Shown in a tile next to the waterfall."); }
            return;
        }
        TetraDemodulatorModule* _this = this;
        switch (id) {
        case S_MEAS:
            if (sectionHeader(id, CONCAT("Measurements##_tetra_meas_", _this->name), ImGuiTreeNodeFlags_DefaultOpen)) {
                if (_this->measDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_mfront_", _this->name))) { _this->focusMeasWin = true; }
                } else {
                    _this->drawMeasWindow(menuWidth);
                }
            }
            break;
        case S_CONST:
            if (sectionHeader(id, CONCAT("Constellation##_tetra_const_", _this->name), ImGuiTreeNodeFlags_DefaultOpen)) {
                if (_this->constDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_cfront_", _this->name))) { _this->focusConstWin = true; }
                } else {
                    _this->drawConstellationOptions("");
                    _this->drawConstellation(menuWidth);
                }
            }
            break;
        case S_TXA:
            if (sectionHeader(id, CONCAT("TX analysis (spectrum)##_tetra_txa_", _this->name))) {
                if (_this->txaDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_tfront_", _this->name))) { _this->focusTxaWin = true; }
                } else {
                    _this->drawTxAnalysis(menuWidth);
                }
            }
            break;
        case S_TREND:
            if (sectionHeader(id, CONCAT("Trends & timeslots##_tetra_trend_", _this->name))) {
                if (_this->trendDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_trfront_", _this->name))) { _this->focusTrendWin = true; }
                } else {
                    _this->drawTrends(menuWidth);
                }
            }
            break;
        case S_CARR:
            if (sectionHeader(id, CONCAT("Carriers (multi-carrier)##_tetra_carr_", _this->name))) {
                if (_this->carrDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_crfront_", _this->name))) { _this->focusCarrWin = true; }
                } else {
                    _this->drawCarriers(menuWidth);
                }
            }
            break;
        case S_BURST:
            if (sectionHeader(id, CONCAT("Burst analysis (uplink)##_tetra_burst_", _this->name))) {
                if (_this->burstDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_bfront_", _this->name))) { _this->focusBurstWin = true; }
                } else {
                    _this->drawBursts(menuWidth);
                }
            }
            break;
        case S_MP:
            if (sectionHeader(id, CONCAT("Multipath (downlink)##_tetra_mp_", _this->name))) {
                if (_this->mpDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_mpfront_", _this->name))) { _this->focusMpWin = true; }
                } else {
                    _this->drawMultipath(menuWidth);
                }
            }
            break;
        case S_ALARM:
            if (sectionHeader(id, CONCAT("Alarms##_tetra_alarm_", _this->name))) { _this->drawAlarms(menuWidth); }
            break;
        case S_REPORT:
            if (sectionHeader(id, CONCAT("Report##_tetra_report_", _this->name))) { _this->drawReport(menuWidth); }
            break;
        case S_GPS:
            if (sectionHeader(id, CONCAT("GPS & coverage map##_tetra_gps_", _this->name))) {
                if (_this->gpsDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_gfront_", _this->name))) { _this->focusGpsWin = true; }
                } else {
                    _this->drawGpsCoverage(menuWidth);
                }
            }
            break;
        case S_NET:
            //OSMO-TETRA: network & cell information (optionally in its own window)
            if (sectionHeader(id, CONCAT("Network & cell##_tetra_net_", _this->name), ImGuiTreeNodeFlags_DefaultOpen)) {
                if (_this->netDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_nfront_", _this->name))) { _this->focusNetWin = true; }
                } else {
                    _this->drawNetworkBody(menuWidth);
                }
            }
            break;
        case S_SUBS:
            _this->drawSubscribers(menuWidth);
            break;
        case S_DUP:
            _this->drawDuplexEditor(menuWidth);
            break;
        default:
            break;
        }
    }

    // ---------------- GPS, cell lock and coverage map ----------------

    void drawGpsCoverage(float menuWidth) {
        ImVec4 good = tc::good();
        ImVec4 warn = tc::warn();
        ImVec4 bad = tc::bad();
        bool running = gps.isRunning();

        // Port selection
        if (running) { style::beginDisabled(); }
        std::string preview = gpsPort.empty() ? "(select GPS port)" : gpsPort;
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetFontSize() * 6);
        if (ImGui::BeginCombo(CONCAT("##_tetra_gps_port_", name), preview.c_str())) {
            for (auto& p : gpsPorts) {
                if (ImGui::Selectable(p.c_str(), p == gpsPort)) {
                    gpsPort = p;
                    config.acquire();
                    config.conf[name]["gpsPort"] = gpsPort;
                    config.release(true);
                }
            }
            if (gpsPorts.empty()) { tw::TextDisabled("no serial ports found"); }
            ImGui::EndCombo();
        }
        if (tw::SameButton(CONCAT("Rescan##_tetra_gps_scan_", name))) { gpsPorts = NmeaGps::listPorts(); }
        static const int bauds[] = { 4800, 9600, 19200, 38400, 57600, 115200 };
        static const char* baudTxt = "4800\0" "9600\0" "19200\0" "38400\0" "57600\0" "115200\0";   // separate literals: "\019" would be an octal escape
        int bi = 1;
        for (int i = 0; i < 6; i++) { if (bauds[i] == gpsBaud) { bi = i; } }
        ImGui::SetNextItemWidth(ImGui::GetFontSize() * 6);
        if (ImGui::Combo(CONCAT("Baud##_tetra_gps_baud_", name), &bi, baudTxt)) {
            gpsBaud = bauds[bi];
            config.acquire();
            config.conf[name]["gpsBaud"] = gpsBaud;
            config.release(true);
        }
        if (running) { style::endDisabled(); }
        tw::SameLineFor(tw::buttonWidth("Disconnect"));
        if (!running) {
            bool can = !gpsPort.empty();
            if (!can) { style::beginDisabled(); }
            if (ImGui::Button(CONCAT("Connect##_tetra_gps_on_", name), ImVec2(-FLT_MIN, 0))) {
                gps.start(gpsPort, gpsBaud);
                config.acquire();
                config.conf[name]["gpsEnabled"] = true;
                config.release(true);
            }
            if (!can) { style::endDisabled(); }
        }
        else {
            if (ImGui::Button(CONCAT("Disconnect##_tetra_gps_off_", name), ImVec2(-FLT_MIN, 0))) {
                gps.stop();
                config.acquire();
                config.conf[name]["gpsEnabled"] = false;
                config.release(true);
            }
        }

        GpsFix f = gps.get();
        std::string err = gps.getError();
        if (!err.empty() && !running) {
            tw::TextColored(bad, "%s", err.c_str());
        }
        else if (running && !f.connected) {
            tw::TextColored(warn, "Waiting for NMEA data (check port/baud rate)");
        }
        else if (running && !f.valid) {
            tw::TextColored(warn, "No GPS fix yet (%d satellites)", f.sats);
        }
        else if (running) {
            tw::TextColored(good, "%.5f, %.5f", f.lat, f.lon);
            tw::SameTextDisabled(" %d sat, HDOP %.1f, %.0f km/h", f.sats, f.hdop, f.speedKmh);
        }

        // Cell lock
        ImGui::Separator();
        TetraMeasurements::CellLock lock = meas.getCellLock();
        bool decLocked = osmotetradecoder.getRxState() == 2 && osmotetradecoder.getMcc() != 0;
        if (!lock.active) {
            if (!decLocked) { style::beginDisabled(); }
            if (ImGui::Button(CONCAT("Lock current cell##_tetra_lock_", name), ImVec2(menuWidth, 0))) {
                lock.active = true;
                lock.mcc = osmotetradecoder.getMcc();
                lock.mnc = osmotetradecoder.getMnc();
                lock.cc = osmotetradecoder.getCc();
                lock.dlFreq = osmotetradecoder.getDlFreq();
                meas.setCellLock(lock);
            }
            if (!decLocked) { style::endDisabled(); }
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Coverage samples are marked 'no service' whenever the decoder\nis not locked to exactly this cell (MCC/MNC/CC/DL frequency).");
            }
        }
        else {
            TetraMeasSnapshot m = meas.get();
            tw::Text("Cell lock:");
            tw::SameTextColored(m.cellMatch == 1 ? good : bad, "%03d/%03d CC %d %.4f MHz", lock.mcc, lock.mnc, lock.cc, lock.dlFreq / 1e6);
            if (ImGui::Button(CONCAT("Unlock cell##_tetra_unlock_", name), ImVec2(menuWidth, 0))) {
                lock.active = false;
                meas.setCellLock(lock);
            }
        }

        // Map export
        ImGui::Separator();
        ImGui::SetNextItemWidth(menuWidth / 2.0f);
        if (ImGui::Combo(CONCAT("Map value##_tetra_map_metric_", name), &mapMetric, "Level\0MER\0BER\0Quality (CRC)\0")) {
            config.acquire();
            config.conf[name]["mapMetric"] = mapMetric;
            config.release(true);
        }
        std::string logPath = meas.getLogPath();
        bool canExport = !logPath.empty() && !mapExport.busy();
        if (!canExport) { style::beginDisabled(); }
        if (ImGui::Button(CONCAT("Export map (KML + OSM)##_tetra_map_exp_", name), ImVec2(menuWidth, 0))) {
            mapExport.crcLimit = evalView.crcLimit;
            mapExport.exportAsync(logPath, (CoverageMapExporter::Metric)mapMetric, meas.getCellLock().active);
        }
        if (!canExport) { style::endDisabled(); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Creates a .kml (Google Earth) and a _map.html (OpenStreetMap)\nnext to the current/last CSV log. Start a CSV log with GPS connected first.");
        }
        if (logPath.empty()) {
            tw::TextDisabled("Start a CSV log (Measurements) to record a drive.");
        }
        std::string st = mapExport.status();
        if (!st.empty()) { ImGui::TextWrapped("%s", st.c_str()); }

        ImGui::Separator();
        if (ImGui::TreeNode(CONCAT("Propagation diagram##_tetra_prop_", name))) {
            if (propView.draw(name, ImGui::GetContentRegionAvail().x, levelCalEff != 0.0f)) {
                config.acquire();
                json& pp = config.conf[name]["prop"];
                pp["bsValid"] = propView.bsValid;
                pp["bsLat"] = propView.bsLat;
                pp["bsLon"] = propView.bsLon;
                pp["metric"] = propView.metric;
                config.release(true);
            }
            ImGui::TreePop();
        }
    }

    // ---------------- Constellation & floating windows ----------------

    void drawConstellationOptions(const char* suffix) {
        if (ImGui::Checkbox(CONCAT(std::string("8 points (pi/4)##_tetra_c8") + suffix + "_", name), &constellation8)) {
            config.acquire();
            config.conf[name]["constellation8"] = constellation8;
            config.release(true);
        }
        tw::SameLineFor(tw::checkboxWidth("Remove I/Q offset"));
        drawOffsetCompCheckbox(suffix);
    }

    // own window per section (icon in the section header)
    bool* detachFlag(int id) {
        switch (id) {
        case S_CONST: return &constDetached;
        case S_MEAS: return &measDetached;
        case S_GPS: return &gpsDetached;
        case S_TXA: return &txaDetached;
        case S_TREND: return &trendDetached;
        case S_CARR: return &carrDetached;
        case S_BURST: return &burstDetached;
        case S_MP: return &mpDetached;
        case S_NET: return &netDetached;
        case S_SUBS: return &subsDetached;
        default: return nullptr;
        }
    }
    void setDetached(int id, bool on) {
        bool* f = detachFlag(id);
        if (!f) { return; }
        if (on && inTiles(id)) { removeFromTiles(id); }
        *f = on;
        if (id == S_MP) { saveMpConfig(); }
        else {
            static const struct { int id; const char* key; } keys[] = { { S_CONST, "constDetached" }, { S_TXA, "txaDetached" }, { S_TREND, "trendDetached" },
                                                                       { S_CARR, "carrDetached" }, { S_BURST, "burstDetached" }, { S_NET, "netDetached" }, { S_SUBS, "subsDetached" },
                                                                       { S_MEAS, "measDetached" }, { S_GPS, "gpsDetached" } };
            for (auto& k : keys) {
                if (k.id != id) { continue; }
                config.acquire();
                config.conf[name][k.key] = on;
                config.release(true);
            }
        }
        if (!on) { return; }
        switch (id) {
        case S_CONST: focusConstWin = true; break;
        case S_TXA: focusTxaWin = true; break;
        case S_TREND: focusTrendWin = true; break;
        case S_CARR: focusCarrWin = true; break;
        case S_BURST: focusBurstWin = true; break;
        case S_MP: focusMpWin = true; break;
        case S_NET: focusNetWin = true; break;
        case S_SUBS: focusSubsWin = true; break;
        case S_MEAS: focusMeasWin = true; break;
        case S_GPS: focusGpsWin = true; break;
        default: break;
        }
    }

    void drawMeasWindow(float w) {
        drawMeasurements(this, w);
        drawVem(w);
    }

    // layout presets: which sections get their own window and where (fractions
    // of the main window's work area); the menu and waterfall stay on the left
    // layout presets: start assignments of the tile column (right of the
    // waterfall) and the strip below it; 0 = classic (everything in the menu)
    void applyLayout(int preset) {
        static const int all[] = { S_CONST, S_MEAS, S_TXA, S_TREND, S_CARR, S_BURST, S_MP, S_NET, S_SUBS, S_GPS };
        for (int id : all) { if (bool* f = detachFlag(id)) { if (*f) { setDetached(id, false); } } }
        tiles.clear();
        tileBottom = false;
        if (preset == 1) {   // measuring station
            tiles = { { S_CONST, 0.42f }, { S_MEAS, 0.38f }, { S_TREND, 0.20f } };
            tileBottom = hamMode;
        }
        else if (preset == 2) {   // drive test
            tiles = { { S_GPS, 0.45f }, { S_MEAS, 0.35f }, { S_TREND, 0.20f } };
        }
        else if (preset == 3) {   // touch: big values first
            tiles = { { S_VALUES, 1.0f }, { S_CONST, 1.0f }, { S_MEAS, 1.0f }, { S_GPS, 1.0f }, { S_NET, 1.0f } };
        }
        tilePreset = preset;
        saveTiles();
        static const char* names[] = { "classic (all in the menu)", "measuring station", "drive test", "touch" };
        meas.addEvent(std::string("Layout: ") + names[std::clamp(preset, 0, 3)]);
    }

    // ---------------- Tile column / strip below the waterfall ----------------

    static constexpr int S_VALUES = 100;   // pseudo section: big value tiles
    struct TileCfg { int sec; float w; };

    bool tileable(int sec) {
        switch (sec) {
        case S_VALUES: case S_CONST: case S_MEAS: case S_TXA: case S_TREND: case S_CARR:
        case S_BURST: case S_MP: case S_GPS: return true;
        case S_NET: return decoder_mode == 0;
        case S_SUBS: return decoder_mode == 0 && hamMode;
        default: return false;
        }
    }
    const char* tileTitle(int sec) {
        switch (sec) {
        case S_VALUES: return "Values (large)";
        case S_CONST: return "Constellation";
        case S_MEAS: return "Measurements";
        case S_TXA: return "TX analysis";
        case S_TREND: return "Trends & timeslots";
        case S_CARR: return "Carriers";
        case S_BURST: return "Burst analysis";
        case S_MP: return "Multipath";
        case S_GPS: return "GPS & coverage";
        case S_NET: return "Network & cell";
        case S_SUBS: return "Stations & Traffic";
        default: return "?";
        }
    }
    // menu header labels (same IDs as in drawSection)
    std::string sectionLabel(int id) {
        switch (id) {
        case S_MEAS: return "Measurements##_tetra_meas_";
        case S_CONST: return "Constellation##_tetra_const_";
        case S_TXA: return "TX analysis (spectrum)##_tetra_txa_";
        case S_TREND: return "Trends & timeslots##_tetra_trend_";
        case S_CARR: return "Carriers (multi-carrier)##_tetra_carr_";
        case S_BURST: return "Burst analysis (uplink)##_tetra_burst_";
        case S_MP: return "Multipath (downlink)##_tetra_mp_";
        case S_GPS: return "GPS & coverage map##_tetra_gps_";
        case S_NET: return "Network & cell##_tetra_net_";
        case S_SUBS: return "Stations & Traffic##_tetra_subs_";
        default: return "?##_tetra_unk_";
        }
    }
    bool bottomActive() { return tileBottom && tileable(S_SUBS); }
    bool tilesActive() { return enabled && (!tiles.empty() || touch::enabled()); }
    bool inTiles(int sec) {
        if (!enabled) { return false; }
        if (sec == S_SUBS && bottomActive()) { return true; }
        if (tiles.empty() && !touch::enabled()) { return false; }
        for (auto& t : tiles) { if (t.sec == sec && tileable(sec)) { return true; } }
        return false;
    }
    void removeFromTiles(int sec) {
        tiles.erase(std::remove_if(tiles.begin(), tiles.end(), [sec](const TileCfg& t) { return t.sec == sec; }), tiles.end());
        if (sec == S_SUBS) { tileBottom = false; }
        saveTiles();
    }

    void saveTiles() {
        config.acquire();
        json& t = config.conf[name]["tiles"];
        t = json::object();
        json arr = json::array();
        for (auto& x : tiles) {
            json e;
            e["sec"] = x.sec == S_VALUES ? std::string("values") : std::string(sectionKey(x.sec));
            e["w"] = x.w;
            arr.push_back(e);
        }
        t["column"] = arr;
        t["width"] = tileColW / style::uiScale;
        t["bottom"] = tileBottom;
        t["bottomHeight"] = tileBottomH / style::uiScale;
        t["preset"] = tilePreset;
        t["touchStep"] = touchStepIdx;
        config.conf[name]["layoutPreset"] = tilePreset;
        config.release(true);
        tilesDirty = false;
    }

    void loadTiles() {
        json& c = config.conf[name];
        if (!c.contains("tiles") || !c["tiles"].is_object()) {
            // from v0.22: the presets opened own windows - turn them into tiles once
            if (c.contains("layoutPreset")) {
                int p = c["layoutPreset"];
                if (p == 1 || p == 2) { tilesMigrate = p; }
            }
            return;
        }
        json& t = c["tiles"];
        tiles.clear();
        if (t.contains("column") && t["column"].is_array()) {
            for (auto& e : t["column"]) {
                if (!e.contains("sec")) { continue; }
                std::string k = e["sec"];
                int sec = -1;
                if (k == "values") { sec = S_VALUES; }
                for (int i = 0; i < S_COUNT; i++) { if (k == sectionKey(i)) { sec = i; } }
                if (sec < 0) { continue; }
                float w = e.contains("w") ? (float)e["w"] : 1.0f;
                tiles.push_back({ sec, std::clamp(w, 0.02f, 50.0f) });
            }
        }
        if (t.contains("width")) { tileColW = (float)t["width"] * style::uiScale; }
        if (t.contains("bottom")) { tileBottom = t["bottom"]; }
        if (t.contains("bottomHeight")) { tileBottomH = (float)t["bottomHeight"] * style::uiScale; }
        if (t.contains("preset")) { tilePreset = t["preset"]; }
        if (t.contains("touchStep")) { touchStepIdx = std::clamp((int)t["touchStep"], 0, 4); }
    }

    // every frame (GUI thread): register / remove the panels in the main window
    void updatePanels() {
        if (tilesMigrate > 0) { int p = tilesMigrate; tilesMigrate = 0; applyLayout(p); }
        { int lr = layoutReq.exchange(-1); if (lr >= 0) { applyLayout(lr); } }
        if (touch::enabled() && enabled && tiles.empty()) { applyLayout(3); }
        bool side = tilesActive();
        bool bottom = enabled && bottomActive() && !touch::enabled();
        if (side != sideRegistered) {
            if (side) { gui::mainWindow.setSidePanel([this]() { drawSidePanel(); }, &tileColW); }
            else { gui::mainWindow.setSidePanel(nullptr, nullptr); }
            sideRegistered = side;
        }
        if (bottom != bottomRegistered) {
            if (bottom) { gui::mainWindow.setBottomPanel([this]() { drawBottomPanel(); }, &tileBottomH); }
            else { gui::mainWindow.setBottomPanel(nullptr, nullptr); }
            bottomRegistered = bottom;
        }
        // splitters of the main window changed a size: save once the mouse is released
        if ((side || bottom) && gui::mainWindow.splitterActive()) { tilesDirty = true; }
        if (tilesDirty && !ImGui::IsMouseDown(ImGuiMouseButton_Left)) { saveTiles(); }
    }
    void removePanels() {
        if (sideRegistered) { gui::mainWindow.setSidePanel(nullptr, nullptr); sideRegistered = false; }
        if (bottomRegistered) { gui::mainWindow.setBottomPanel(nullptr, nullptr); bottomRegistered = false; }
    }

    void drawTileBody(int sec, float w, float h) {
        switch (sec) {
        case S_VALUES: drawBigValues(w); break;
        case S_CONST: {
            drawConstellationOptions("t");
            float side = (std::min)(w, (std::max)(80.0f * style::uiScale, h - ImGui::GetFrameHeightWithSpacing() * 1.5f));
            float x0 = ImGui::GetCursorPosX();
            if (w > side) { ImGui::SetCursorPosX(x0 + (w - side) * 0.5f); }
            drawConstellation(side);
            break;
        }
        case S_MEAS: drawMeasWindow(w); break;
        case S_TXA: drawTxAnalysis(w); break;
        case S_TREND: drawTrends(w); break;
        case S_CARR: drawCarriers(w); break;
        case S_BURST: drawBursts(w); break;
        case S_MP: { bool was = mpRun; drawMultipath(w); if (was != mpRun) { saveMpConfig(); } break; }
        case S_GPS: drawGpsCoverage(w); break;
        case S_NET: drawNetworkBody(w); break;
        case S_SUBS: drawSubscribersBody(w, 0); break;
        default: break;
        }
    }

    // tile header: section choice, close; returns false if the tile was closed
    bool tileHeader(int i) {
        std::string id = std::to_string(i) + "_" + name;
        float bw = ImGui::GetFrameHeight();
        ImGui::SetNextItemWidth((std::max)(60.0f, ImGui::GetContentRegionAvail().x - bw - ImGui::GetStyle().ItemSpacing.x));
        if (ImGui::BeginCombo(("##_tile_sec_" + id).c_str(), tileTitle(tiles[i].sec))) {
            static const int order[] = { S_VALUES, S_CONST, S_MEAS, S_NET, S_SUBS, S_TXA, S_TREND, S_CARR, S_BURST, S_MP, S_GPS };
            for (int sec : order) {
                if (!tileable(sec)) { continue; }
                bool used = false;
                for (size_t k = 0; k < tiles.size(); k++) { if (k != (size_t)i && tiles[k].sec == sec) { used = true; } }
                if (sec == S_SUBS && bottomActive()) { used = true; }
                if (ImGui::Selectable(tileTitle(sec), tiles[i].sec == sec, used ? ImGuiSelectableFlags_Disabled : 0)) {
                    if (bool* f = detachFlag(sec)) { if (*f) { setDetached(sec, false); } }
                    tiles[i].sec = sec;
                    saveTiles();
                }
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button(("x##_tile_close_" + id).c_str(), ImVec2(bw, bw))) {
            tiles.erase(tiles.begin() + i);
            saveTiles();
            return false;
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Close this tile (the section goes back into the menu)"); }
        return true;
    }

    void addTile() {
        static const int order[] = { S_CONST, S_MEAS, S_NET, S_TREND, S_GPS, S_TXA, S_CARR, S_BURST, S_MP, S_SUBS, S_VALUES };
        for (int sec : order) {
            if (!tileable(sec) || inTiles(sec)) { continue; }
            if (bool* f = detachFlag(sec)) { if (*f) { setDetached(sec, false); } }
            float w = 0.0f;
            for (auto& t : tiles) { w += t.w; }
            tiles.push_back({ sec, tiles.empty() ? 1.0f : w / tiles.size() });
            saveTiles();
            return;
        }
    }

    void drawSidePanel() {
        if (touch::enabled()) { drawTouchPanel(); return; }
        const float split = 6.0f * style::uiScale;
        // tool row
        if (ImGui::SmallButton(CONCAT("+ Tile##_tile_add_", name))) { addTile(); }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Add a tile below (choose its content in the tile header)"); }
        if (hamMode && decoder_mode == 0) {
            ImGui::SameLine();
            bool b = tileBottom;
            if (ImGui::Checkbox(CONCAT("Stations below##_tile_bottom_", name), &b)) {
                if (b) { removeFromTiles(S_SUBS); if (subsDetached) { setDetached(S_SUBS, false); } }
                tileBottom = b;
                saveTiles();
            }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Stations & Traffic as a wide strip below the waterfall"); }
        }
        ImGui::SameLine();
        if (ImGui::SmallButton(CONCAT("Reset##_tile_reset_", name))) { applyLayout(tilePreset); }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Back to the chosen layout preset (Layout... in the TETRA menu)"); }
        // drop sections that are not available in the current mode (e.g. NETSYMS)
        std::vector<int> vis;
        for (size_t i = 0; i < tiles.size(); i++) { if (tileable(tiles[i].sec)) { vis.push_back((int)i); } }
        if (vis.empty()) {
            tw::TextDisabled("No tiles. Add one with '+ Tile' or choose a\nlayout preset (Layout... in the TETRA menu).");
            return;
        }
        float avail = ImGui::GetContentRegionAvail().y - split * (vis.size() - 1);
        float wsum = 0.0f;
        for (int i : vis) { wsum += tiles[i].w; }
        const float minH = ImGui::GetFrameHeightWithSpacing() * 2.0f;
        for (size_t n = 0; n < vis.size(); n++) {
            int i = vis[n];
            float h = (std::max)(minH, avail * tiles[i].w / wsum);
            if (n + 1 == vis.size()) { h = 0.0f; }   // last one takes the rest
            std::string cid = "##_tile_" + std::to_string(i) + "_" + name;
            ImGui::BeginChild(cid.c_str(), ImVec2(0, h), true, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
            bool alive = tileHeader(i);
            if (alive) {
                ImGui::BeginChild((cid + "_body").c_str(), ImVec2(0, 0), false);
                ImVec2 a = ImGui::GetContentRegionAvail();
                drawTileBody(tiles[i].sec, a.x, a.y);
                touch::dragScroll();
                ImGui::EndChild();
            }
            ImGui::EndChild();
            if (!alive) { return; }   // indices changed: redraw next frame
            if (n + 1 < vis.size()) {
                int j = vis[n + 1];
                ImGui::InvisibleButton((cid + "_split").c_str(), ImVec2(ImGui::GetContentRegionAvail().x, split));
                bool hov = ImGui::IsItemHovered(), act = ImGui::IsItemActive();
                if (hov || act) { ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS); }
                ImGui::GetWindowDrawList()->AddRectFilled(ImGui::GetItemRectMin(), ImGui::GetItemRectMax(),
                                                         ImGui::GetColorU32(act ? ImGuiCol_SeparatorActive : hov ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator));
                if (act) {
                    float dy = ImGui::GetIO().MouseDelta.y;
                    float dw = dy / (std::max)(1.0f, avail) * wsum;
                    float lo = minH / (std::max)(1.0f, avail) * wsum;
                    if (tiles[i].w + dw < lo) { dw = lo - tiles[i].w; }
                    if (tiles[j].w - dw < lo) { dw = tiles[j].w - lo; }
                    tiles[i].w += dw;
                    tiles[j].w -= dw;
                    tilesDirty = true;
                }
            }
        }
    }

    void drawBottomPanel() {
        ImVec2 a = ImGui::GetContentRegionAvail();
        tw::Text("Stations & Traffic");
        ImGui::SameLine();
        if (ImGui::SmallButton(CONCAT("x##_tile_bclose_", name))) { tileBottom = false; saveTiles(); return; }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Close the strip (back into the menu)"); }
        ImGui::BeginChild(CONCAT("##_tile_bottom_body_", name), ImVec2(0, 0), false);
        drawSubscribersBody(a.x, 0);
        touch::dragScroll();
        ImGui::EndChild();
    }

    // ---------------- Touch layout: tabs, large values, frequency keys ----------------

    void drawBigValues(float w) {
        TetraMeasSnapshot m = meas.get();
        bool cal = levelCalEff != 0.0f;
        GpsFix g = gps.isRunning() ? gps.get() : GpsFix();
        char v[6][48];
        snprintf(v[0], 48, m.valid ? (cal ? "%.1f dBm" : "%.1f dBFS") : "--", m.levelDbfs + (cal ? levelCalEff : 0.0f));
        snprintf(v[1], 48, (m.valid && m.symSync) ? "%.1f dB" : "--", m.merDb);
        snprintf(v[2], 48, (m.valid && m.symSync) ? "%.1f %%" : "--", m.evmPct);
        if (m.absErrValid) { snprintf(v[3], 48, "%+.0f Hz", m.absErrHz); }
        else if (m.valid && m.symSync) { snprintf(v[3], 48, "%+.0f Hz*", m.carrierOffsetHz); }
        else { snprintf(v[3], 48, "--"); }
        snprintf(v[4], 48, "%d", (int)subscribers.snapshot().size());
        if (!gps.isRunning()) { snprintf(v[5], 48, "off"); }
        else if (!g.valid) { snprintf(v[5], 48, "no fix (%d sat)", g.sats); }
        else { snprintf(v[5], 48, "fix, %d sat", g.sats); }
        static const char* keys[6] = { "Level", "MER", "Decoder EVM", "Freq. error", "Stations", "GPS" };
        static const char* tips[6] = { nullptr, nullptr, nullptr,
                                       "Carrier frequency against the frequency announced by the cell (SYSINFO).\n* = against the TETRA VFO only (no SYSINFO yet).",
                                       "Stations seen since the start (amateur radio mode).",
                                       "GPS receiver: fix and satellites." };
        int cols = w > 360.0f * style::uiScale ? 2 : 1;
        float cw = (w - (cols - 1) * ImGui::GetStyle().ItemSpacing.x) / cols;
        for (int k = 0; k < 6; k++) {
            if (k % cols != 0) { ImGui::SameLine(); }
            ImGui::BeginChild((std::string("##_bigv_") + std::to_string(k) + name).c_str(), ImVec2(cw, ImGui::GetFontSize() * 4.2f), true, ImGuiWindowFlags_NoScrollbar);
            tw::TextDisabled("%s", keys[k]);
            float bs = ImGui::GetFrameHeight() * 1.2f;
            ImGui::SameLine(ImGui::GetContentRegionAvail().x + ImGui::GetCursorPosX() - bs - ImGui::GetStyle().ItemSpacing.x);
            ImGui::SmallButton((std::string("i##_bigvi_") + std::to_string(k) + name).c_str());
            bool tipHere = ImGui::IsItemHovered() || ImGui::IsItemActive();
            if (tipHere) {
                if (k == 0) { ImGui::SetTooltip("%s", tetra_help::text(tetra_help::LEVEL)); }
                else if (k == 1 || k == 2) { ImGui::SetTooltip("%s", tetra_help::text(tetra_help::MER)); }
                else { ImGui::SetTooltip("%s", tips[k]); }
            }
            ImGui::PushFont(style::bigFont);
            ImGui::TextColored(tc::val(), "%s", v[k]);
            ImGui::PopFont();
            ImGui::EndChild();
        }
    }

    void drawTouchPanel() {
        static const double steps[5] = { 6250.0, 12500.0, 25000.0, 100000.0, 1000000.0 };
        static const char* stepTxt = "6.25 kHz\0" "12.5 kHz\0" "25 kHz\0" "100 kHz\0" "1 MHz\0";
        float bh = ImGui::GetFrameHeight() * 1.6f;
        // frequency keys
        if (vfo) {
            double f = mainVfoFreq();
            double st = steps[std::clamp(touchStepIdx, 0, 4)];
            if (ImGui::Button(CONCAT("-##_touch_fdn_", name), ImVec2(bh * 1.3f, bh))) { tuner::tune(tuner::TUNER_MODE_NORMAL, name, std::round(f / st) * st - st); }
            ImGui::SameLine();
            ImGui::PushFont(style::bigFont);
            ImGui::Text("%.4f", f / 1e6);
            ImGui::PopFont();
            ImGui::SameLine();
            if (ImGui::Button(CONCAT("+##_touch_fup_", name), ImVec2(bh * 1.3f, bh))) { tuner::tune(tuner::TUNER_MODE_NORMAL, name, std::round(f / st) * st + st); }
            ImGui::SameLine();
            ImGui::SetNextItemWidth((std::max)(70.0f * style::uiScale, ImGui::GetContentRegionAvail().x));
            if (ImGui::Combo(CONCAT("##_touch_step_", name), &touchStepIdx, stepTxt)) { saveTiles(); }
        }
        std::vector<int> vis;
        for (size_t i = 0; i < tiles.size(); i++) { if (tileable(tiles[i].sec)) { vis.push_back((int)i); } }
        if (vis.empty()) { return; }
        touchTab = std::clamp(touchTab, 0, (int)vis.size() - 1);
        float tabH = ImGui::GetFrameHeight() * 1.8f;
        ImGui::BeginChild(CONCAT("##_touch_body_", name), ImVec2(0, -(tabH + ImGui::GetStyle().ItemSpacing.y)), true);
        ImVec2 a = ImGui::GetContentRegionAvail();
        drawTileBody(tiles[vis[touchTab]].sec, a.x, a.y);
        touch::dragScroll();
        ImGui::EndChild();
        // tabs at the bottom
        float tw_ = (ImGui::GetContentRegionAvail().x - (vis.size() - 1) * ImGui::GetStyle().ItemSpacing.x) / vis.size();
        for (size_t n = 0; n < vis.size(); n++) {
            if (n > 0) { ImGui::SameLine(); }
            bool act = (int)n == touchTab;
            if (act) { ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive)); }
            if (ImGui::Button((std::string(tileTitle(tiles[vis[n]].sec)) + "##_touch_tab_" + std::to_string(n) + name).c_str(), ImVec2(tw_, tabH))) { touchTab = (int)n; }
            if (act) { ImGui::PopStyleColor(); }
        }
    }

    void drawConstellation(float width) {
        TetraMeasSnapshot m = meas.get();
        // with the offset removed from the display, judge the colour by the
        // offset-free EVM as well (EVM = 10^(-MER/20))
        float evm = compOriginOffset ? 100.0f * powf(10.0f, -m.merExclDb / 20.0f) : m.evmPct;
        bool good = m.valid && m.symSync && evm <= TetraConstellationView::RMS_LIMIT * 100.0f;
        constView.legendEvm = m.valid ? evm : NAN;
        constView.legendWhat = compOriginOffset ? "EVM w/o I/Q offset" : "EVM";
        // Colours follow the theme if it defines "Constellation*" keys (e.g. "RoSc light", "RoSc dark")
        {
            TetraConstellationView::Palette p;
            ImVec4 c;
            auto take = [&](const char* key, ImU32& dst) {
                if (gui::themeManager.getExtraColor(key, c)) { dst = ImGui::ColorConvertFloat4ToU32(c); }
            };
            take("ConstellationBg", p.bg);
            take("ConstellationBorder", p.border);
            take("ConstellationGrid", p.grid);
            take("ConstellationPeakZone", p.peakZone);
            take("ConstellationRmsZone", p.rmsZone);
            take("ConstellationRmsRing", p.rmsRing);
            take("ConstellationMarker", p.marker);
            take("ConstellationDotGood", p.dotGood);
            take("ConstellationDotBad", p.dotBad);
            take("ConstellationText", p.text);
            constView.pal = p;
        }
        constView.draw(CONCAT("##_tetra_cview_", name), ImVec2(width, width), constellation8, good, m.symSync);
    }

    void drawOffsetCompCheckbox(const char* suffix = "") {
        if (ImGui::Checkbox(CONCAT(std::string("Remove I/Q offset##_tetra_ccomp_") + suffix, name), &compOriginOffset)) {
            config.acquire();
            config.conf[name]["compOriginOffset"] = compOriginOffset;
            config.release(true);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Display only: centers the point clouds by removing the constant\n"
                              "I/Q (origin) offset, like 'compensate origin offset' on a VSA.\n"
                              "MER/EVM and the I/Q offset value are still measured without it.");
        }
    }


    // ---------------- Network & cell (SYNC, SYSINFO, MLE broadcast) ----------------

    static const char* cellLoadName(int l) {
        switch (l & 3) { case 1: return "low"; case 2: return "medium"; case 3: return "high"; default: return "not given"; }
    }

    // days since 1970-01-01 for a civil date (proleptic Gregorian)
    static int64_t daysFromCivil(int64_t y, unsigned m, unsigned d) {
        y -= m <= 2;
        const int64_t era = (y >= 0 ? y : y - 399) / 400;
        const unsigned yoe = (unsigned)(y - era * 400);
        const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
        const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
        return era * 146097 + (int64_t)doe - 719468;
    }

    static std::string fmtUtc(int64_t t) {
        int64_t days = t >= 0 ? t / 86400 : (t - 86399) / 86400;
        int64_t sod = t - days * 86400;
        // civil from days
        int64_t z = days + 719468;
        int64_t era = (z >= 0 ? z : z - 146096) / 146097;
        unsigned doe = (unsigned)(z - era * 146097);
        unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
        int64_t y = (int64_t)yoe + era * 400;
        unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
        unsigned mp = (5 * doy + 2) / 153;
        unsigned d = doy - (153 * mp + 2) / 5 + 1;
        unsigned m = mp < 10 ? mp + 3 : mp - 9;
        y += (m <= 2);
        char buf[48];
        snprintf(buf, sizeof(buf), "%04lld-%02u-%02u %02d:%02d:%02d", (long long)y, m, d,
                 (int)(sod / 3600), (int)((sod / 60) % 60), (int)(sod % 60));
        return buf;
    }

    void drawCellDetails(float menuWidth) {
        const tetra_display_state* d = osmotetradecoder.getDisplay();
        if (!d) { return; }
        ImVec4 val = tc::val();
        auto row = [&](const char* label, const char* tip, const char* fmt, auto... args) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::TextUnformatted(label);
            if (tip && ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", tip); }
            ImGui::TableSetColumnIndex(1);
            tw::TextColored(val, fmt, args...);
            if (tip && ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", tip); }
        };
        ImGuiTableFlags tf = ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg;

        if (ImGui::TreeNode(CONCAT("Cell parameters (SYNC / SYSINFO)##_tetra_cellp_", name))) {
            if (!d->sync_valid && !d->sysinfo_valid) { tw::TextDisabled("no SYNC / SYSINFO received yet"); }
            else if (ImGui::BeginTable(CONCAT("##_tetra_cellt_", name), 2, tf)) {
                static const float offKHz[4] = { 0.0f, 6.25f, -6.25f, 12.5f };
                static const char* sharing[4] = { "continuous transmission", "carrier sharing", "MCCH sharing", "traffic carrier sharing" };
                static const int resFrames[8] = { 1, 2, 3, 4, 6, 9, 12, 18 };
                if (d->sysinfo_valid) {
                    row("Main carrier", "Carrier number of the main carrier (MCCH), from SYSINFO.", "%d (band %d, offset %+.2f kHz)",
                        d->main_carrier, d->freq_band, offKHz[d->freq_offset & 3]);
                    row("Location area (LA)", "Location area of this cell. Moving to a cell with another LA\nmakes the radios register again.", "%d", d->la);
                    row("Subscriber classes", "16-bit mask of subscriber classes allowed to access this cell (decimal).", "%u", (unsigned)d->subscr_class);
                    row("MS max TX power", "Maximum transmit power a radio may use in this cell.", "%d dBm", d->ms_txpwr_max ? 10 + 5 * d->ms_txpwr_max : 0);
                    row("Min RX access level", "A radio needs at least this downlink level to use the cell\n(RXLEV_ACCESS_MIN).", "%d dBm", -125 + 5 * d->rxlev_access_min);
                    row("Access parameter", "Reference for open-loop power control of the radios on access\n(ACCESS_PARAMETER).", "%d dBm", -53 + 2 * d->access_parameter);
                    if (d->radio_dl_timeout) {
                        row("Radio DL timeout", "Time without decodable downlink after which a radio gives up\nthe cell (multiples of 144 timeslots).", "%d x 144 slots (%.1f s)",
                            d->radio_dl_timeout, d->radio_dl_timeout * 144 * (85.0 / 6.0) / 1000.0);
                    } else {
                        row("Radio DL timeout", "Time without decodable downlink after which a radio gives up the cell.", "%s", "disabled");
                    }
                    row("Secondary control ch.", "Number of common secondary control channels (CSCH) in use.", "%d", d->num_csch);
                    if (d->cck_valid) { row("CCK identifier", "Common cipher key in use (air interface encryption, security class 3).", "%d", d->cck_id); }
                }
                if (d->sync_valid) {
                    row("Cell load (SYNC)", "Cell service level from D-MLE-SYNC.", "%s", cellLoadName(d->cell_load_sync));
                    row("Air interface", "System code from SYNC: 0 = ETS 300 392-2 ed. 1, 1 = EN 300 392-2 V2.3.2+,\n2 = V3.1.1+ (Release 2), 10..15 = DMO.", "code %d", d->system_code);
                    row("Sharing mode", "Whether the carrier / MCCH is shared between several cells.", "%s", sharing[d->sharing_mode & 3]);
                    if (d->sharing_mode == 2) {
                        row("Reserved frames", "MCCH sharing: frames reserved for this cell per two multiframes.", "%d", resFrames[d->ts_reserved_frames & 7]);
                    }
                    row("U-plane DTX", "Discontinuous transmission allowed on the traffic channel.", "%s", d->uplane_dtx ? "yes" : "no");
                    row("Frame 18 extension", "Radios may use frame 18 for extra signalling.", "%s", d->frame18_ext ? "yes" : "no");
                    row("Late entry", "Late entry information is sent for ongoing group calls.", "%s", d->late_entry ? "supported" : "no");
                    row("Neighbour broadcast", "D-MLE-SYNC: D-NWRK-BROADCAST sent / enquiry supported.", "%s / %s",
                        (d->nb_cell_bcast & 2) ? "yes" : "no", (d->nb_cell_bcast & 1) ? "yes" : "no");
                }
                ImGui::EndTable();
            }
            ImGui::TreePop();
        }

        if (ImGui::TreeNode(CONCAT("Network broadcast & neighbour cells##_tetra_nb_", name))) {
            int64_t now = (int64_t)time(nullptr);
            if (!d->nwrk_count) {
                tw::TextDisabled("no D-NWRK-BROADCAST received yet");
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Sent by the network every few seconds to minutes, if at all\n(see 'Neighbour broadcast' in the cell parameters).");
                }
            } else {
                tw::Text("Received: %u, last %llds ago", d->nwrk_count, (long long)(now - d->nwrk_rx_time));
                if (tw::SameSmallButton(CONCAT("Clear##_tetra_nbclr_", name))) { osmotetradecoder.clearNeighbours(); }
                if (ImGui::BeginTable(CONCAT("##_tetra_nwt_", name), 2, tf)) {
                    if (d->reselect_valid) {
                        row("Slow reselect", "Threshold / hysteresis for normal cell reselection (2 dB steps).", "%d dB / %d dB", 2 * d->slow_resel_thr, 2 * d->slow_resel_hyst);
                        row("Fast reselect", "Threshold / hysteresis for fast (urgent) cell reselection.", "%d dB / %d dB", 2 * d->fast_resel_thr, 2 * d->fast_resel_hyst);
                        row("Cell load", "Cell load from D-NWRK-BROADCAST.", "%s", cellLoadName(d->cell_load_nwrk));
                    }
                    if (d->nettime_valid) {
                        int64_t base = daysFromCivil(2000 + d->nettime_year, 1, 1) * 86400 + (int64_t)d->nettime_utc * 2;
                        int64_t netNow = base + (now - d->nettime_rx_time);
                        int offMin = (d->nettime_sign ? -1 : 1) * d->nettime_offset * 15;
                        row("Network time", "TETRA network time (UTC, 2 s resolution), advanced by the age of the message.", "%s UTC", fmtUtc(netNow).c_str());
                        row("Local offset", "Local time offset announced by the network.", "%+03d:%02d", offMin / 60, std::abs(offMin) % 60);
                        row("vs. PC clock", "Network time minus PC clock at reception (2 s resolution).", "%+lld s", (long long)(base - d->nettime_rx_time));
                    }
                    ImGui::EndTable();
                }
                if (d->n_neigh > 0) {
                    tw::Text("Neighbour cells: %d", d->n_neigh);
                    if (ImGui::BeginTable(CONCAT("##_tetra_neigh_", name), 7, tf | ImGuiTableFlags_Borders | ImGuiTableFlags_Resizable)) {
                        ImGui::TableSetupColumn("Cell");
                        ImGui::TableSetupColumn("DL MHz");
                        ImGui::TableSetupColumn("LA");
                        ImGui::TableSetupColumn("Load");
                        ImGui::TableSetupColumn("Min RX");
                        ImGui::TableSetupColumn("Age");
                        ImGui::TableSetupColumn("");
                        ImGui::TableHeadersRow();
                        for (int i = 0; i < d->n_neigh && i < 16; i++) {
                            const auto& n = d->neigh[i];
                            int band = n.has_ext ? n.band : d->freq_band;
                            int off = n.has_ext ? n.offset : d->freq_offset;
                            double f = (double)tetra_dl_carrier_hz((uint8_t)band, (uint16_t)n.main_carrier, (uint8_t)off);
                            ImGui::TableNextRow();
                            ImGui::TableSetColumnIndex(0);
                            tw::Text("%d%s", n.cell_id, n.synced ? " (sync)" : "");
                            if (ImGui::IsItemHovered()) {
                                ImGui::SetTooltip("Cell identifier %d, carrier %d%s\nMCC %s, MNC %s, max MS TX %s",
                                                  n.cell_id, n.main_carrier, n.synced ? ", synchronised to this cell" : "",
                                                  n.mcc >= 0 ? std::to_string(n.mcc).c_str() : "-",
                                                  n.mnc >= 0 ? std::to_string(n.mnc).c_str() : "-",
                                                  n.txpwr > 0 ? (std::to_string(10 + 5 * n.txpwr) + " dBm").c_str() : "-");
                            }
                            ImGui::TableSetColumnIndex(1);
                            tw::TextColored(val, "%.4f", f / 1e6);
                            ImGui::TableSetColumnIndex(2);
                            if (n.la >= 0) { tw::Text("%d", n.la); } else { tw::TextDisabled("-"); }
                            ImGui::TableSetColumnIndex(3);
                            ImGui::TextUnformatted(cellLoadName(n.load));
                            ImGui::TableSetColumnIndex(4);
                            if (n.rxlev_min >= 0) { tw::Text("%d dBm", -125 + 5 * n.rxlev_min); } else { tw::TextDisabled("-"); }
                            ImGui::TableSetColumnIndex(5);
                            tw::Text("%llds", (long long)(now - n.rx_time));
                            ImGui::TableSetColumnIndex(6);
                            if (f > 1e6 && ImGui::SmallButton(CONCAT("Tune##_tetra_ntune_" + std::to_string(i) + "_", name))) {
                                tuner::tune(tuner::TUNER_MODE_NORMAL, name, f);
                            }
                        }
                        ImGui::EndTable();
                    }
                }
            }
            ImGui::TreePop();
        }
    }

    void drawNetworkBody(float menuWidth) {
        TetraDemodulatorModule* _this = this;
        ImGui::BoxIndicator(ImGui::GetFontSize()*2, _this->symbolExtractor.sync ? IM_COL32(5, 230, 5, 255) : IM_COL32(230, 5, 5, 255));
        tw::SameText(" Sync");
        //OSMO-TETRA
        int dec_st = _this->osmotetradecoder.getRxState();
        ImGui::BoxIndicator(ImGui::GetFontSize()*2, (dec_st == 0) ? IM_COL32(230, 5, 5, 255) : ((dec_st == 2) ? IM_COL32(5, 230, 5, 255) : IM_COL32(230, 230, 5, 255)));
        tw::SameText(" Decoder:  %s", (dec_st == 0) ? "Unlocked" : ((dec_st == 2) ? "Locked" : "Know next start"));
        if(dec_st != 2) {
            style::beginDisabled();
        }
        tw::Text("Hyperframe: "); tw::SameTextColored(tc::val(), "%05d", _this->osmotetradecoder.getCurrHyperframe()); tw::SameText(" | Multiframe: "); tw::SameTextColored(tc::val(), "%02d", _this->osmotetradecoder.getCurrMultiframe()); tw::SameText("| Frame: "); tw::SameTextColored(tc::val(), "%02d", _this->osmotetradecoder.getCurrFrame());
        tw::Text("Timeslots: ");
        for(int i = 0; i < 4; i++) {
            switch(_this->osmotetradecoder.getTimeslotContent(i)) {
                case 0:
                    tw::SameTextColored(tc::lgrey(), "   UL  ");
                    break;
                case 1:
                    tw::SameTextColored(tc::magenta(), " DATA  ");
                    break;
                case 2:
                    tw::SameTextColored(tc::val(), "  NDB  ");
                    break;
                case 3:
                    tw::SameTextColored(tc::cyan(), " SYNC  ");
                    break;
                case 4:
                    tw::SameTextColored(tc::good(), " VOICE ");
                    break;
            }
        }
        int crc_failed = _this->osmotetradecoder.getLastCrcFail();
        if(crc_failed) {
            ImGui::BoxIndicator(ImGui::GetFontSize()*2, IM_COL32(230, 5, 5, 255));
            tw::SameText(" CRC: "); tw::SameTextColored(tc::bad(), "FAIL");
            style::beginDisabled();
        } else {
            ImGui::BoxIndicator(ImGui::GetFontSize()*2, IM_COL32(5, 230, 5, 255));
            tw::SameText(" CRC: "); tw::SameTextColored(tc::good(), "PASS");
        }
        int dl_usg = _this->osmotetradecoder.getDlUsage();
        int ul_usg = _this->osmotetradecoder.getUlUsage();
        tw::Text("DL:");tw::SameTextColored(tc::val(), "%7.4f", ((float)_this->osmotetradecoder.getDlFreq()/1000000.0f));tw::SameText(" MHz ");tw::SameTextColored(tc::val(), (dl_usg == 0 ? "Unalloc" : (dl_usg == 1 ? "Assigned ctl" : (dl_usg == 2 ? "Common ctl" : (dl_usg == 3 ? "Reserved" : "Traffic")))));
        tw::Text("UL:");tw::SameTextColored(tc::val(), "%7.4f", ((float)_this->osmotetradecoder.getUlFreq()/1000000.0f));tw::SameText(" MHz ");tw::SameTextColored(tc::val(), (ul_usg == 0 ? "Unalloc" : "Traffic"));
        tw::Text("Duplex: ");tw::SameTextColored(tc::val(), "Table %d, Band %d%s",
            _this->osmotetradecoder.getDuplexIdx(), _this->osmotetradecoder.getFreqBand(),
            _this->osmotetradecoder.getReverseOp() ? ", reverse" : "");
        tw::Text("Access1: ");tw::SameTextColored(tc::val(), "%c", _this->osmotetradecoder.getAccess1Code());tw::SameText("/");tw::SameTextColored(tc::val(), "%d", _this->osmotetradecoder.getAccess1());tw::SameText("| Access2: ");tw::SameTextColored(tc::val(), "%c", _this->osmotetradecoder.getAccess2Code());tw::SameText("/");tw::SameTextColored(tc::val(), "%d", _this->osmotetradecoder.getAccess2());
        tw::Text("MCC: ");tw::SameTextColored(tc::val(), "%03d", _this->osmotetradecoder.getMcc());tw::SameText("| MNC: ");tw::SameTextColored(tc::val(), "%03d", _this->osmotetradecoder.getMnc());tw::SameText("| CC: ");tw::SameTextColored(tc::val(), "%d", _this->osmotetradecoder.getCc());
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Colour code (0..63) from the SYNC PDU = %d.\nIt is part of the scrambling code: if it were wrong,\nno other channel would pass its CRC.", _this->osmotetradecoder.getCc());
        }
        ImVec4 on_color = tc::good();
        ImVec4 off_color = tc::bad();
        tw::TextColored(_this->osmotetradecoder.getAdvancedLink() ? on_color : off_color, "Adv. link  ");tw::SameTextColored(_this->osmotetradecoder.getAirEncryption() ? on_color : off_color, "Encryption  ");tw::SameTextColored(_this->osmotetradecoder.getSndcpData() ? on_color : off_color, "SNDCP");
        tw::TextColored(_this->osmotetradecoder.getCircuitData() ? on_color : off_color, "Circuit data  ");tw::SameTextColored(_this->osmotetradecoder.getVoiceService() ? on_color : off_color, "Voice  ");tw::SameTextColored(_this->osmotetradecoder.getNormalMode() ? on_color : off_color, "Normal mode");
        tw::TextColored(_this->osmotetradecoder.getMigrationSupported() ? on_color : off_color, "Migration  ");tw::SameTextColored(_this->osmotetradecoder.getNeverMinimumMode() ? on_color : off_color, "Never min mode  ");tw::SameTextColored(_this->osmotetradecoder.getPriorityCell() ? on_color : off_color, "Priority cell");
        tw::TextColored(_this->osmotetradecoder.getDeregMandatory() ? on_color : off_color, "Dereg req.  ");tw::SameTextColored(_this->osmotetradecoder.getRegMandatory() ? on_color : off_color, "Reg req.");
        if(crc_failed) {
            style::endDisabled();
        }
        if(dec_st != 2) {
            style::endDisabled();
        }
        drawCellDetails(menuWidth);
    }

    // ---------------- TX analysis (spectrum, adjacent channel power) ----------------

    void txaLoop() {
        if (!enabled || !vfo || !txaRun) {
            if (txa.isRunning()) { txa.stop(); }
            return;
        }
        double off = vfo->getOffset();   // same value as the other users of the shared tap
        if (!txa.isRunning()) {
            txa.setFftSize(4096 << txaFftIdx);
            txa.setAverages(txaAvg);
            txa.start(off);
        } else {
            txa.setOffset(off);
        }
        TetraMeasSnapshot m = meas.get();
        txa.setCarrierOffsetHz((m.valid && m.symSync) ? m.carrierOffsetHz : 0.0);
    }

    void saveTxaConfig() {
        config.acquire();
        config.conf[name]["txaRun"] = txaRun;
        config.conf[name]["txaFft"] = txaFftIdx;
        config.conf[name]["txaAvg"] = txaAvg;
        config.conf[name]["txaLimits"] = json::array({ txaLimit[0], txaLimit[1], txaLimit[2] });
        config.release(true);
    }

    void drawTxaSpectrum(const TxAnalysisResult& r, float width, double rxDcHz) {
        const float h = 230.0f * style::uiScale;
        ImGuiWindow* window = ImGui::GetCurrentWindow();
        if (window->SkipItems) { return; }
        ImVec2 p0 = window->DC.CursorPos;
        ImRect bb(p0, ImVec2(p0.x + width, p0.y + h));
        ImGui::ItemSize(ImVec2(width, h));
        if (!ImGui::ItemAdd(bb, ImGui::GetID(CONCAT("##_tetra_txaplot_", name)))) { return; }
        // colours follow the constellation palette of the theme
        TetraConstellationView::Palette pal = constView.pal;
        ImDrawList* dl = window->DrawList;
        dl->AddRectFilled(bb.Min, bb.Max, pal.bg);
        dl->AddRect(bb.Min, bb.Max, pal.border);
        const float lm = 34.0f * style::uiScale, bm = 16.0f * style::uiScale;
        ImRect pr(ImVec2(bb.Min.x + lm, bb.Min.y + 4), ImVec2(bb.Max.x - 6, bb.Max.y - bm));
        const double span = 100000.0;
        double top = std::ceil((r.inbandDensityDb + 10.0) / 10.0) * 10.0;
        double bot = top - 100.0;
        auto X = [&](double f) { return (float)(pr.Min.x + (f + span) / (2.0 * span) * pr.GetWidth()); };
        auto Y = [&](double db) {
            double t = (top - db) / (top - bot);
            t = std::clamp(t, 0.0, 1.0);
            return (float)(pr.Min.y + t * pr.GetHeight());
        };
        char buf[32];
        for (double db = top; db >= bot - 0.1; db -= 10.0) {
            dl->AddLine(ImVec2(pr.Min.x, Y(db)), ImVec2(pr.Max.x, Y(db)), pal.grid);
            snprintf(buf, sizeof(buf), "%.0f", db);
            dl->AddText(ImVec2(bb.Min.x + 3, Y(db) - ImGui::GetFontSize() * 0.5f), pal.text, buf);
        }
        for (double f = -span; f <= span + 1; f += 25000.0) {
            dl->AddLine(ImVec2(X(f), pr.Min.y), ImVec2(X(f), pr.Max.y), pal.grid);
            snprintf(buf, sizeof(buf), "%+.0f", f / 1000.0);
            dl->AddText(ImVec2(X(f) - 10, pr.Max.y + 1), pal.text, f == 0 ? "0" : buf);
        }
        // adjacent channel windows
        for (int i = 0; i < 6; i++) {
            double c = TxMath::ACP_OFF[i];
            double hw = (1.0 + TxMath::ALPHA) * TxMath::RS / 2.0;
            dl->AddRectFilled(ImVec2(X(c - hw), pr.Min.y), ImVec2(X(c + hw), pr.Max.y), (pal.rmsZone & 0x00FFFFFF) | 0x18000000);
        }
        dl->PushClipRect(pr.Min, pr.Max, true);
        const int n = (int)r.psdDb.size();
        auto curve = [&](const std::vector<float>& v, ImU32 col, float th) {
            if ((int)v.size() != n || n < 2) { return; }
            // one point per pixel column: max of the bins (keeps narrow lines visible)
            int cols = (int)pr.GetWidth();
            ImVec2 prev;
            bool have = false;
            for (int c = 0; c < cols; c++) {
                double fa = -span + 2.0 * span * c / cols, fb = -span + 2.0 * span * (c + 1) / cols;
                int ka = (int)std::floor(fa / r.binHz) + n / 2, kb = (int)std::ceil(fb / r.binHz) + n / 2;
                ka = std::clamp(ka, 0, n - 1); kb = std::clamp(kb, ka, n - 1);
                float mx = -400.0f;
                for (int k = ka; k <= kb; k++) { mx = (std::max)(mx, v[k]); }
                ImVec2 pt(pr.Min.x + c, Y(mx));
                if (have) { dl->AddLine(prev, pt, col, th); }
                prev = pt;
                have = true;
            }
        };
        curve(r.floorDb, IM_COL32(140, 140, 140, 200), 1.0f);
        // ideal TETRA spectrum, scaled to the measured in-band density
        {
            ImVec2 prev;
            bool have = false;
            for (double f = -13000; f <= 13000; f += 250) {
                double w = TxMath::rcWeight(f);
                double db = w > 1e-12 ? r.inbandDensityDb + 10.0 * std::log10(w) : bot;
                ImVec2 pt(X(f), Y(db));
                if (have) { dl->AddLine(prev, pt, pal.dotGood, 1.0f); }
                prev = pt;
                have = true;
            }
        }
        curve(r.psdDb, ImGui::GetColorU32(ImGuiCol_PlotLines), 1.2f * style::uiScale);
        if (std::fabs(rxDcHz) < span) {
            dl->AddLine(ImVec2(X(rxDcHz), pr.Min.y), ImVec2(X(rxDcHz), pr.Max.y), pal.dotBad, 1.0f);
            dl->AddText(ImVec2(X(rxDcHz) + 3, pr.Min.y + 2), pal.dotBad, "RX DC");
        }
        dl->PopClipRect();
        snprintf(buf, sizeof(buf), "RBW %.0f Hz", r.rbwHz);
        dl->AddText(ImVec2(pr.Max.x - ImGui::CalcTextSize(buf).x - 4, pr.Min.y + 2), pal.text, buf);
    }

    void drawTxAnalysis(float menuWidth) {
        if (ImGui::Checkbox(CONCAT("25 kHz channel raster in the spectrum##_tetra_raster_", name), &chanRaster)) {
            config.acquire();
            config.conf[name]["chanRaster"] = chanRaster;
            config.release(true);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Dashed lines at +-12.5 kHz and +-37.5 kHz around the TETRA channel:\n"
                              "the own channel and the adjacent channels (ACP measurement).\n"
                              "Smoothing of the spectrum: Display menu -> FFT Smoothing.");
        }
        if (ImGui::Checkbox(CONCAT("Run analysis (250 kHz tap)##_tetra_txarun_", name), &txaRun)) { saveTxaConfig(); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("250 kHz wide tap around the TETRA channel (288 kS/s, not shown on the waterfall).\n"
                              "Shared with modulation accuracy and multipath: one tap at the full SDR\n"
                              "rate instead of one per analysis.");
        }
        if (!txaRun) {
            tw::TextDisabled("Spectrum, adjacent channel power, occupied bandwidth,\nshoulders and carrier leakage of the received carrier.");
            return;
        }
        ImGui::SetNextItemWidth(menuWidth * 0.45f);
        const char* rbws = "122 Hz (4k FFT)\0" "61 Hz (8k FFT)\0" "31 Hz (16k FFT)\0";
        if (ImGui::Combo(CONCAT("RBW##_tetra_txarbw_", name), &txaFftIdx, rbws)) {
            txa.setFftSize(4096 << txaFftIdx);
            saveTxaConfig();
        }
        tw::SameLineFor(menuWidth * 0.3f + tw::textWidth("Avg") + ImGui::GetStyle().ItemInnerSpacing.x);
        ImGui::SetNextItemWidth(menuWidth * 0.3f);
        if (ImGui::SliderInt(CONCAT("Avg##_tetra_txaavg_", name), &txaAvg, 1, 100)) {
            txa.setAverages(txaAvg);
            saveTxaConfig();
        }
        if (ImGui::SmallButton(CONCAT("Reset average##_tetra_txarst_", name))) { txa.resetAverage(); }
        if (tw::SameSmallButton(CONCAT("Store floor##_tetra_txaflr_", name))) { txa.storeFloor(); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Stores the current averaged spectrum as the receiver's measurement floor.\n"
                              "Do this with the transmitter off or the SDR input terminated (50 ohm),\n"
                              "same gain and sample rate. Results close to the floor are marked.\n"
                              "Without a stored floor, the noise far from the carrier is used.");
        }
        if (txa.hasFloor()) {
            if (tw::SameSmallButton(CONCAT("Clear floor##_tetra_txaflc_", name))) { txa.clearFloor(); }
        }

        TxAnalysisResult r = txa.get();
        TetraMeasSnapshot m = meas.get();
        ImVec4 good = tc::good(), warn = tc::warn(), bad = tc::bad(), val = tc::val();

        // receiver DC position relative to the channel
        double centerOff = vfo ? vfo->wtfVFO->centerOffset : 0.0;
        double rxDcHz = -centerOff;   // the SDR's own DC, seen from the channel centre
        if (std::fabs(centerOff) < 15000.0) {
            tw::TextColored(bad, "Channel is on the receiver's centre frequency.");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("The SDR's own DC offset lands on the carrier: I/Q offset and carrier\n"
                                  "leakage then include the receiver. Tune the SDR so that the channel\n"
                                  "is at least ~100 kHz away from the centre.");
            }
        } else if (std::fabs(rxDcHz) < 90000.0) {
            tw::TextColored(warn, "Receiver DC at %+.0f kHz falls into the analysis span.", rxDcHz / 1000.0);
        }
        if (!r.valid) {
            tw::TextDisabled("collecting samples ...");
            return;
        }
        if (r.peakDbfs > -3.0) {
            tw::TextColored(bad, "Tap peak %.1f dBFS: overload risk, reduce RF gain.", r.peakDbfs);
        }

        drawTxaSpectrum(r, menuWidth, rxDcHz);
        tw::TextDisabled("measured spectrum, ideal TETRA spectrum (thin), stored floor (grey), ACP windows (shaded)");

        // Adjacent channel power table
        auto acpCell = [&](double v, double floorDb, float limit) {
            bool limited = v - floorDb < 3.0;
            ImVec4 c = limited ? tc::grey() : (v <= limit ? good : bad);
            tw::TextColored(c, "%6.1f%s", v, limited ? " *" : "");
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip(limited ? "Within 3 dB of the measurement floor (%.1f dBc):\nthe true value may be lower (receiver limited)."
                                          : "Measurement floor %.1f dBc", floorDb);
            }
        };
        tw::Text("Channel power:"); tw::SameTextColored(val, "%.1f dBFS", r.chanPowerDbfs);
        tw::SameTextDisabled("(%d avg)", r.averages);
        if (ImGui::BeginTable(CONCAT("##_tetra_acp_", name), 5, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("ACP");
            ImGui::TableSetupColumn("lower dBc");
            ImGui::TableSetupColumn("upper dBc");
            ImGui::TableSetupColumn("limit");
            ImGui::TableSetupColumn("floor");
            ImGui::TableHeadersRow();
            const char* lbl[3] = { "+-25 kHz", "+-50 kHz", "+-75 kHz" };
            for (int i = 0; i < 3; i++) {
                int lo = 2 - i, hi = 3 + i;
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0); ImGui::TextUnformatted(lbl[i]);
                ImGui::TableSetColumnIndex(1); acpCell(r.acpDbc[lo], r.acpFloorDbc[lo], txaLimit[i]);
                ImGui::TableSetColumnIndex(2); acpCell(r.acpDbc[hi], r.acpFloorDbc[hi], txaLimit[i]);
                ImGui::TableSetColumnIndex(3); tw::Text("%.0f", txaLimit[i]);
                ImGui::TableSetColumnIndex(4); tw::TextDisabled("%.0f", (std::max)(r.acpFloorDbc[lo], r.acpFloorDbc[hi]));
            }
            ImGui::EndTable();
        }
        if (ImGui::TreeNode(CONCAT("ACP limits##_tetra_acplim_", name))) {
            bool ch = false;
            ImGui::SetNextItemWidth(menuWidth * 0.3f); ch |= ImGui::InputFloat(CONCAT("25 kHz##_tetra_l25_", name), &txaLimit[0], 0, 0, "%.0f dBc");
            ImGui::SetNextItemWidth(menuWidth * 0.3f); ch |= ImGui::InputFloat(CONCAT("50 kHz##_tetra_l50_", name), &txaLimit[1], 0, 0, "%.0f dBc");
            ImGui::SetNextItemWidth(menuWidth * 0.3f); ch |= ImGui::InputFloat(CONCAT("75 kHz##_tetra_l75_", name), &txaLimit[2], 0, 0, "%.0f dBc");
            // presets: EN 300 394-1 7.1.3.2, tables 7.1 to 7.3 (normal test conditions)
            tw::Text("Preset:");
            if (tw::SameSmallButton(CONCAT("BS / MS class 1-3##_tetra_lp1_", name))) { txaLimit[0] = -60.0f; txaLimit[1] = -70.0f; txaLimit[2] = -70.0f; ch = true; }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Table 7.2 (below 700 MHz): -60 / -70 / -70 dBc"); }
            if (tw::SameSmallButton(CONCAT("MS class 4/4L##_tetra_lp2_", name))) { txaLimit[0] = -55.0f; txaLimit[1] = -70.0f; txaLimit[2] = -70.0f; ch = true; }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Table 7.1 (below 700 MHz): -55 / -70 / -70 dBc"); }
            if (tw::SameSmallButton(CONCAT("> 700 MHz##_tetra_lp3_", name))) { txaLimit[0] = -55.0f; txaLimit[1] = -65.0f; txaLimit[2] = -65.0f; ch = true; }
            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Table 7.3: -55 / -65 / -65 dBc (75 kHz: -70 dBc for BS classes 1-3 and MS 1/1L)"); }
            if (ch) { saveTxaConfig(); }
            tw::TextDisabled("Average power over the useful part through the TETRA filter (RRC 0.35)\n"
                             "at +-25 / 50 / 75 kHz (EN 300 394-1 7.1.3). Never stricter than -36 dBm.\n"
                             "Own floor of the test system must be below -70 / -80 / -80 dBc (B.1.2.3).");
            ImGui::TreePop();
        }

        // Bandwidth and spectral shape
        tw::Text("Occupied BW (99%%):"); tw::SameTextColored(std::fabs(r.obwHz - r.obwIdealHz) < 1500.0 ? good : warn, "%.2f kHz", r.obwHz / 1000.0);
        tw::SameTextDisabled("(ideal %.2f)", r.obwIdealHz / 1000.0);
        tw::Text("-26 dB bandwidth:"); tw::SameTextColored(std::fabs(r.bw26Hz - r.bw26IdealHz) < 2000.0 ? good : warn, "%.2f kHz", r.bw26Hz / 1000.0);
        tw::SameTextDisabled("(ideal %.2f)", r.bw26IdealHz / 1000.0);
        tw::Text("Shoulders (13.5-20 kHz):"); tw::SameNext();
        {
            auto shCol = [&](double v) { return (v - r.shoulderFloorDb < 3.0) ? tc::grey() : (v <= -45.0 ? good : (v <= -35.0 ? warn : bad)); };
            tw::TextColored(shCol(r.shoulderLoDb), "%.1f", r.shoulderLoDb); tw::SameText("/"); tw::SameTextColored(shCol(r.shoulderHiDb), "%.1f dB", r.shoulderHiDb);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Spectral density just outside the TETRA spectrum, relative to the in-band density.\n"
                                  "An ideal transmitter has nothing there. Raised shoulders = spectral regrowth from\n"
                                  "non-linearity (PA compression, DAC clipping, too high digital level).\n"
                                  "Floor here: %.1f dB. Different lower/upper values point to I/Q imbalance or IMD asymmetry.",
                                  r.shoulderFloorDb);
            }
        }
        tw::Text("Edge slope (-6..-30 dB):"); tw::SameNext();
        if (r.slopeLoDbPerKHz > 0 && r.slopeHiDbPerKHz > 0) {
            tw::TextColored(val, "%.1f / %.1f dB/kHz", r.slopeLoDbPerKHz, r.slopeHiDbPerKHz);
            tw::SameTextDisabled("(ideal %.1f)", r.slopeIdealDbPerKHz);
        } else {
            tw::TextDisabled("-- (not enough dynamic range)");
        }

        // Carrier leakage / DC: spectral line and constellation
        tw::Text("Carrier line (spectrum):"); tw::SameNext();
        if (r.carrierLeakBelow) { tw::TextColored(good, "< %.1f dBc", r.carrierLeakDbc); }
        else { tw::TextColored(r.carrierLeakDbc <= -30.0 ? good : (r.carrierLeakDbc <= -20.0 ? warn : bad), "%.1f dBc", r.carrierLeakDbc); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Unmodulated line at the carrier (transmitter LO leakage), found with a 3.8 Hz\n"
                              "resolution FFT. Detection limit about -35 dBc; the constellation value below\n"
                              "is far more sensitive.");
        }
        tw::Text("I/Q offset (constellation):"); tw::SameNext();
        if (m.valid && m.symSync) {
            tw::TextColored(m.iqOffsetDb <= -30.0f ? good : (m.iqOffsetDb <= -20.0f ? warn : bad), "%.1f dB", m.iqOffsetDb);
        } else { tw::TextDisabled("--"); }
        // diagnosis: where does a strong carrier component come from?
        if (m.valid && m.symSync && m.iqOffsetDb > -30.0f) {
            if (std::fabs(rxDcHz) < 15000.0) {
                ImGui::TextColored(warn, "%s", "Cause unclear: the receiver DC is inside the channel (see above).");
            }
            else if (!r.carrierLeakBelow && std::fabs(r.carrierLeakDbc - m.iqOffsetDb) <= 4.0) {
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + menuWidth);
                ImGui::TextColored(m.iqOffsetDb > -20.0f ? bad : warn,
                    "Carrier leakage of the transmitter: spectrum and constellation agree (%.1f / %.1f dB) and the receiver DC is %+.0f kHz away. Check the TX LO / DC calibration.",
                    r.carrierLeakDbc, m.iqOffsetDb, rxDcHz / 1000.0);
                ImGui::PopTextWrapPos();
            }
            else if (!r.carrierLeakBelow) {
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + menuWidth);
                ImGui::TextColored(warn, "%s", "Carrier component: spectrum and constellation disagree. Check the level (overload?) and measure again.");
                ImGui::PopTextWrapPos();
            }
        }
        tw::Text("Noise density:"); tw::SameTextDisabled("%.1f dBFS/bin, in-band %.1f dBFS/bin", r.noiseDensityDb, r.inbandDensityDb);
    }

    // ---------------- Trends, timeslots, further carriers, bursts (v0.18) ----------------

    double mainVfoFreq() { return vfo ? gui::waterfall.getCenterFrequency() + vfo->getOffset() : 0.0; }

    // neighbour cells announced by the network (label, DL frequency)
    std::vector<std::pair<std::string, double>> neighbourCarriers() {
        std::vector<std::pair<std::string, double>> out;
        if (decoder_mode != 0) { return out; }
        const struct tetra_display_state* d = osmotetradecoder.getDisplay();
        if (!d) { return out; }
        for (int i = 0; i < d->n_neigh && i < 16; i++) {
            const auto& n = d->neigh[i];
            int band = n.has_ext ? n.band : d->freq_band;
            int off = n.has_ext ? n.offset : d->freq_offset;
            double f = (double)tetra_dl_carrier_hz((uint8_t)band, (uint16_t)n.main_carrier, (uint8_t)off);
            if (f > 1e6) { out.push_back({ "Cell " + std::to_string(n.cell_id), f }); }
        }
        return out;
    }

    void saveCarriers() {
        config.acquire();
        json arr = json::array();
        for (int i = 0; i < carrierMon.count(); i++) {
            json c;
            c["label"] = carrierMon.at(i).label;
            c["freq"] = carrierMon.at(i).freqHz;
            arr.push_back(c);
        }
        config.conf[name]["carriers"] = arr;
        config.release(true);
    }

    void saveBurstConfig() {
        config.acquire();
        json& b = config.conf[name]["burst"];
        b["run"] = burstRun;
        b["source"] = burstSt.source;
        b["mhz"] = burstSt.manualMHz;
        b["avg"] = burstSt.averages;
        b["ripple"] = burstSt.maskRippleDb;
        b["off"] = burstSt.maskOffDbc;
        b["ramp"] = burstSt.maskRampSym;
        b["stdMask"] = burstSt.stdMask;
        b["station"] = burstSt.station;
        b["msClass4"] = burstSt.msClass4;
        b["msPowerKnown"] = burstSt.msPowerKnown;
        b["msPowerDbm"] = burstSt.msPowerDbm;
        b["timing"] = burstSt.timing;
        b["ulOnlyV222"] = true;
        b["linStepDb"] = burstSt.linStepDb;
        b["transients"] = burstSt.transients;
        b["floorSpan"] = burstSt.floorSpan;
        config.release(true);
    }

    void saveTrendConfig() {
        config.acquire();
        config.conf[name]["trendSpan"] = trendView.spanIdx;
        json arr = json::array();
        for (int i = 0; i < TetraTrendView::P_COUNT; i++) { arr.push_back(trendView.show[i]); }
        config.conf[name]["trendShow"] = arr;
        config.release(true);
    }

    // every GUI frame
    static double nz(float v) { return std::isnan(v) ? -999.0 : (double)v; }   // JSON has no NaN

    void extraLoops() {
        if (reportReq.exchange(false)) {
            std::string r = makeReport();
            std::lock_guard<std::mutex> lck(reportMtx);
            reportResult = r;
        }
        double pc = pendingCarrierHz.exchange(0.0);
        if (pc > 1e6) { carrierMon.add("", pc); }
        carrierMon.update(enabled && vfo != nullptr);
        bandMon.loop(enabled && vfo != nullptr);
        if (carrierMon.takeChanged()) { saveCarriers(); }
        double f = 0.0;
        if (burstSt.source == 0) { f = (double)meas.get().ulFreq; }
        else if (burstSt.source == 1) { f = burstSt.manualMHz * 1e6; }
        else { f = mainVfoFreq(); }
        double dlTap = (burstSt.timing && burstSt.source != 2) ? mainVfoFreq() : 0.0;
        burstAn.setKnownCarrier(burstSt.source == 2 && osmotetradecoder.getRxState() == 2);
        burstAn.loop(enabled && burstRun, f, dlTap);
        chanAn.loop(enabled && mpRun && vfo != nullptr, mainVfoFreq());
        {
            TetraMeasSnapshot vm = meas.get();
            vemAn.loop(enabled && vemRun && vfo != nullptr && decoder_mode == 0, mainVfoFreq(), (uint16_t)vm.mcc, (uint16_t)vm.mnc, (uint8_t)vm.cc);
        }
        alarmLoop();
        {
            // summary for the automation channel (read from another thread)
            std::string st = burstAn.status;
            for (auto& ch : st) { if (ch == '"' || ch == '\\') { ch = '\''; } }
            int run = 0, syn = 0;
            for (int i = 0; i < carrierMon.count(); i++) {
                if (carrierMon.at(i).valid) { run++; }
                if (carrierMon.at(i).valid && carrierMon.at(i).sync) { syn++; }
            }
            std::lock_guard<std::mutex> lck(dbgMtx);
            burstStatus = st;
            dbgCarr[0] = carrierMon.count(); dbgCarr[1] = run; dbgCarr[2] = syn;
            dbgAlarmTotal = alarms.total();
            dbgAlarmActive = alarms.any();
        }
    }

    void drawTrends(float width) {
        if (trendView.fileMode) {
            tw::Text("Log:");
            tw::SameTextColored(tc::val(), "%s", trendView.fileName.c_str());
            if (tw::SameSmallButton(CONCAT("Back to live##_tetra_trlive_", name))) {
                trendView.fileMode = false;
                trendView.filePts.clear();
                trendView.invalidate();
            }
        }
        tw::Text("Open a CSV log:");
        std::string reqPath;
        int tabReq = -1;
        {
            std::lock_guard<std::mutex> lck(trendReqMtx);
            reqPath.swap(trendLoadReq);
            tabReq = trendTabReq;
            trendTabReq = -1;
        }
        if (trendFile.render("##_tetra_trfile_" + name) || !reqPath.empty()) {
            std::string path = reqPath.empty() ? trendFile.expandString(trendFile.path) : reqPath;
            if (!reqPath.empty() || trendFile.pathIsValid()) {
                std::vector<TetraTrendPoint> pts;
                if (TetraTrendStore::loadCsv(path, pts, trendFileError)) {
                    trendView.filePts = std::move(pts);
                    trendView.fileMode = true;
                    trendView.fileName = std::filesystem::path(path).filename().string();
                    trendView.spanIdx = TetraTrendView::SPAN_N - 1;   // whole log
                    trendView.invalidate();
                    trendFileError.clear();
                }
            }
        }
        if (!trendFileError.empty()) { tw::TextColored(tc::bad(), "%s", trendFileError.c_str()); }
        ImGui::SetNextItemWidth((std::min)(width, 100.0f * style::uiScale));
        if (ImGui::Combo(CONCAT("Time span##_tetra_trspan_", name), &trendView.spanIdx, TetraTrendView::spanNames())) { saveTrendConfig(); }
        {
            static const char* labels[TetraTrendView::P_COUNT] = { "Level", "MER", "CRC errors", "Carrier offset", "Timeslots", "Bursts/s" };
            std::string ids[TetraTrendView::P_COUNT];
            tw::CheckRow row;
            for (int i = 0; i < TetraTrendView::P_COUNT; i++) {
                ids[i] = std::string(labels[i]) + "##_tetra_trp" + std::to_string(i) + "_" + name;
                row.add(ids[i].c_str(), &trendView.show[i]);
            }
            if (row.draw("Show:") >= 0) { trendView.invalidate(); saveTrendConfig(); }
        }
        if (ImGui::BeginTabBar(CONCAT("##_tetra_trtabs_", name))) {
            if (ImGui::BeginTabItem(CONCAT("Over time##_tetra_trt0_", name), nullptr, tabReq == 0 ? ImGuiTabItemFlags_SetSelected : 0)) {
                trendView.draw("##_tetra_trplot_" + name, width, meas.trend(), constView.pal);
                if (!trendView.fileMode) {
                    tw::TextDisabled("Last 24 h kept in memory; longer: Start CSV log (Measurements)\nand open the file here later.");
                    drawTimeslotTable(width);
                }
                ImGui::EndTabItem();
            }
            bool evalTab = false;
            if (ImGui::BeginTabItem(CONCAT("Frequency stability##_tetra_trt1_", name), nullptr, tabReq == 1 ? ImGuiTabItemFlags_SetSelected : 0)) {
                evalTab = true;
                updateEval();
                evalView.drawStability(name, width, constView.pal);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(CONCAT("Level vs. quality##_tetra_trt2_", name), nullptr, tabReq == 2 ? ImGuiTabItemFlags_SetSelected : 0)) {
                evalTab = true;
                updateEval();
                bool ch = false;
                evalView.drawQuality(name, width, constView.pal, ch);
                if (ch) {
                    config.acquire();
                    config.conf[name]["crcLimit"] = evalView.crcLimit;
                    config.release(true);
                }
                ImGui::EndTabItem();
            }
            (void)evalTab;
            ImGui::EndTabBar();
        }
    }

    void updateEval() {
        if (trendView.fileMode) { evalView.update(trendView.filePts, evalFile != trendView.fileName); evalFile = trendView.fileName; }
        else {
            if (!evalFile.empty()) { evalFile.clear(); evalView.update(meas.trend().copy(0), true); }
            else { evalView.update(meas.trend().copy(0), false); }
        }
    }

    void drawTimeslotTable(float width) {
        if (decoder_mode != 0) { return; }
        TetraMeasSnapshot m = meas.get();
        if (!m.valid) { return; }
        tw::Text("Timeslot occupancy since reset");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("From the ACCESS-ASSIGN PDU (AACH) of every downlink burst, frames 1-17\n"
                              "(frame 18 is always control). 'Reset totals' under Measurements\n"
                              "starts the statistics again.");
        }
        ImGuiTableFlags tf = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_ScrollX;
        float th = ImGui::GetTextLineHeightWithSpacing() * 6.4f;
        if (!ImGui::BeginTable(CONCAT("##_tetra_tstab_", name), 7, tf, ImVec2(width, th))) { return; }
        tw::CellScope cells;
        ImGui::TableSetupScrollFreeze(1, 1);
        ImGui::TableSetupColumn("TS");
        ImGui::TableSetupColumn("Traffic");
        ImGui::TableSetupColumn("Assigned");
        ImGui::TableSetupColumn("Common");
        ImGui::TableSetupColumn("Free");
        ImGui::TableSetupColumn("UL traffic");
        ImGui::TableSetupColumn("Now");
        ImGui::TableHeadersRow();
        static const char* catName[4] = { "free", "assigned ctrl", "common ctrl", "traffic" };
        ImVec4 val = tc::val();
        for (int t = 0; t < 4; t++) {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            ImGui::Text("TS%d", t + 1);
            uint32_t n = 0, nu = 0;
            for (int c = 0; c < 4; c++) { n += m.dlUseTotal[t][c]; }
            for (int c = 0; c < 3; c++) { nu += m.ulUseTotal[t][c]; }
            if (n == 0) {
                ImGui::TableSetColumnIndex(1);
                ImGui::TextDisabled("--");
                continue;
            }
            int colOf[4] = { 4, 2, 3, 1 };
            for (int c = 0; c < 4; c++) {
                ImGui::TableSetColumnIndex(colOf[c]);
                ImGui::TextColored(c == 3 ? val : tc::lgrey(), "%.1f %%", 100.0 * m.dlUseTotal[t][c] / n);
            }
            ImGui::TableSetColumnIndex(5);
            if (nu) { ImGui::TextColored(tc::lgrey(), "%.1f %%", 100.0 * m.ulUseTotal[t][2] / nu); } else { ImGui::TextDisabled("--"); }
            ImGui::TableSetColumnIndex(6);
            int best = -1;
            uint32_t bn = 0;
            for (int c = 0; c < 4; c++) { if (m.dlUse[t][c] > bn) { bn = m.dlUse[t][c]; best = c; } }
            if (best >= 0) { ImGui::TextColored(best == 3 ? tc::good() : tc::lgrey(), "%s", catName[best]); } else { ImGui::TextDisabled("--"); }
        }
        ImGui::EndTable();
    }

    void drawCarriers(float width) {
        TetraMeasSnapshot m = meas.get();
        bool ok = m.valid;
        carrierMon.draw(name, width, constView.pal, mainVfoFreq(), ok ? m.levelDbfs : NAN, ok ? m.merDb : NAN,
                        ok && m.symSync, levelCalEff, neighbourCarriers());
        if (carrierMon.takeChanged()) { saveCarriers(); }
    }

    void drawBursts(float width) {
        TetraMeasSnapshot m = meas.get();
        bool run = burstRun;
        SourceGainInterface* g = findGainInterface();
        bool gainHeld = !autoGain || (gainHolding && !gainFollowNow()) || !(g && g->gainControllable());
        if (burstAn.draw(name, width, constView.pal, burstSt, (double)m.ulFreq, mainVfoFreq(), levelCalEff, gainHeld, run)) {
            burstRun = run;
            saveBurstConfig();
        }
        burstRun = run;
    }

    // ---- alarms (once per measurement second) ----
    void alarmLoop() {
        TetraMeasSnapshot m = meas.get();
        if (!m.valid || m.timestamp == lastAlarmT) { return; }
        lastAlarmT = m.timestamp;
        TetraAlarms::Input in;
        in.valid = enabled;
        in.level = m.levelDbfs + m.levelCalDb;
        if (m.rxState == 2 && m.symSync) { in.mer = m.merDb; }
        double ok = 0, fail = 0;
        for (int i = 0; i < 4; i++) { ok += m.crcOk[i]; fail += m.crcFail[i]; }
        if (ok + fail > 0) { in.crcFail = (float)(100.0 * fail / (ok + fail)); }
        in.lockLost = decoder_mode == 0 && (m.rxState != 2 || m.syncLost > 0);
        if (burstRun && burstAn.isRunning()) {
            TetraBurstResult br = burstAn.get();
            if (!std::isnan(br.gapDbfs)) { in.ulNoise = br.gapDbfs + m.levelCalDb; }
        }
        if (m.resyncSlip > slipLast) { lossThisSecond = true; }
        slipLast = m.resyncSlip;
        in.dataLoss = lossThisSecond;
        lossThisSecond = false;
        bool raised = false;
        auto ev = alarms.evaluate(alarmSt, in, raised);
        for (auto& e : ev) { meas.addEvent(e); }
        if (raised && alarmSt.autoLog && !meas.isLogging() && logFolder.pathIsValid()) {
            std::string err;
            if (meas.startLog(logFolder.expandString(logFolder.path), err)) { meas.addEvent("CSV log started by alarm"); }
        }
    }

    void saveAlarmConfig() {
        config.acquire();
        json& a = config.conf[name]["alarms"];
        a["enabled"] = alarmSt.enabled; a["levelOn"] = alarmSt.levelOn; a["levelBelow"] = alarmSt.levelBelow;
        a["merOn"] = alarmSt.merOn; a["merBelow"] = alarmSt.merBelow; a["crcOn"] = alarmSt.crcOn; a["crcAbove"] = alarmSt.crcAbove;
        a["syncOn"] = alarmSt.syncOn; a["ulOn"] = alarmSt.ulOn; a["ulAbove"] = alarmSt.ulAbove;
        a["holdS"] = alarmSt.holdS; a["beep"] = alarmSt.beep; a["autoLog"] = alarmSt.autoLog; a["lossOn"] = alarmSt.lossOn;
        config.release(true);
    }

    void saveMpConfig() {
        config.acquire();
        json& mp = config.conf[name]["mp"];
        mp["run"] = mpRun;
        mp["mem"] = mpMem;
        mp["detached"] = mpDetached;
        config.release(true);
    }

    // channel raster: dashed boundaries of the TETRA channel and its neighbours
    // (+-12.5 kHz, +-37.5 kHz) in the spectrum
    static void fftRedraw(ImGui::WaterFall::FFTRedrawArgs args, void* ctx) {
        TetraDemodulatorModule* _this = (TetraDemodulatorModule*)ctx;
        if (!_this->enabled || !_this->chanRaster || !_this->vfo) { return; }
        if (args.freqToPixelRatio * 25000.0 < 8.0) { return; }   // zoomed out too far
        double fc = _this->mainVfoFreq();
        ImDrawList* dl = args.window->DrawList;
        ImVec4 tc4 = ImGui::GetStyleColorVec4(ImGuiCol_Text);
        tc4.w *= 0.45f;
        ImU32 col = ImGui::ColorConvertFloat4ToU32(tc4);
        float dash = 4.0f * style::uiScale;
        for (int k = -3; k <= 3; k += 2) {
            double f = fc + k * 12500.0;
            if (f < args.lowFreq || f > args.highFreq) { continue; }
            float x = (float)std::round(args.min.x + (f - args.lowFreq) * args.freqToPixelRatio) + 0.5f;
            for (float y = args.min.y; y < args.max.y; y += 2.0f * dash) {
                dl->AddLine(ImVec2(x, y), ImVec2(x, (std::min)(y + dash, args.max.y)), col, 1.0f);
            }
        }
    }

    // modulation accuracy per EN 300 394-1 10.1.3 (own measurement path)
    void drawVem(float width) {
        if (decoder_mode != 0) { return; }
        ImGui::Separator();
        bool run = vemRun;
        SourceGainInterface* g = findGainInterface();
        bool gainHeld = !autoGain || (gainHolding && !gainFollowNow()) || !(g && g->gainControllable());
        if (vemAn.draw(name, width, run, gainHeld, freqRef)) {
            vemRun = run;
            config.acquire();
            config.conf[name]["vemRun"] = vemRun;
            config.release(true);
        }
    }

    void drawMultipath(float width) {
        bool run = mpRun;
        int mem = mpMem;
        bool ch = chanAn.draw(name, width, constView.pal, run, mem);
        mpRun = run;
        mpMem = mem;
        if (ch) { saveMpConfig(); }
    }

    void drawAlarms(float width) {
        if (TetraAlarms::draw(name, width, alarmSt, levelCalDb != 0.0f)) { saveAlarmConfig(); }
        if (alarmSt.enabled) {
            if (alarms.any()) { tw::TextColored(tc::bad(), "Active: %s", alarms.activeText().c_str()); }
            else { tw::TextColored(tc::good(), "No alarm"); }
            tw::SameTextDisabled("(%d raised)", alarms.total());
        }
    }

    // ---- report ----
    std::string makeReport() {
        TetraReportData d;
        d.edition = EDITION_NAME;
        d.version = EDITION_VERSION;
        d.notes = reportNotes;
        core::configManager.acquire();
        std::string res = core::configManager.conf.contains("resourcesDirectory") ? core::configManager.conf["resourcesDirectory"].get<std::string>() : std::string();
        core::configManager.release();
        if (!res.empty()) { d.logoPath = res + "/icons/sdrpp.png"; }
        TetraMeasSnapshot m = meas.get();
        d.cellValid = m.mcc != 0;
        d.mcc = m.mcc; d.mnc = m.mnc; d.cc = m.cc;
        d.dlFreq = m.dlFreq; d.ulFreq = m.ulFreq; d.vfoFreq = mainVfoFreq();
        d.levelCalibrated = levelCalDb != 0.0f;
        int span = TetraTrendView::spanSeconds(trendView.spanIdx);
        d.trend = trendView.fileMode ? trendView.filePts : meas.trend().copy(span > 0 ? time(nullptr) - span : 0);
        if (trendView.fileMode) { d.levelCalibrated = !d.trend.empty() && d.trend.front().calibrated; }   // unit of the loaded log
        memcpy(d.dlUseTotal, m.dlUseTotal, sizeof(d.dlUseTotal));
        memcpy(d.ulUseTotal, m.ulUseTotal, sizeof(d.ulUseTotal));
        d.syncLostTotal = m.syncLostTotal;
        for (int i = 0; i < 4; i++) { d.crcOkTotal += m.crcOkTotal[i]; d.crcFailTotal += m.crcFailTotal[i]; }
        d.berTotal = m.berTotal;
        TxAnalysisResult r = txa.get();
        if (txaRun && r.valid) {
            d.txaValid = true;
            d.txaChan = r.chanPowerDbfs;
            for (int i = 0; i < 6; i++) { d.txaAcp[i] = r.acpDbc[i]; d.txaAcpFloor[i] = r.acpFloorDbc[i]; }
            d.txaObw = r.obwHz; d.txaObwIdeal = r.obwIdealHz; d.txaBw26 = r.bw26Hz; d.txaBw26Ideal = r.bw26IdealHz;
            d.txaLeak = r.carrierLeakDbc; d.txaLeakBelow = r.carrierLeakBelow;
        }
        if (burstRun && burstAn.isRunning()) {
            d.burstValid = true;
            d.burst = burstAn.get();
            d.burstFreq = burstSt.source == 0 ? (double)m.ulFreq : burstSt.source == 1 ? burstSt.manualMHz * 1e6 : mainVfoFreq();
        }
        if (mpRun && chanAn.isRunning()) {
            TetraChannelResult c = chanAn.get();
            d.mpValid = c.valid;
            d.mp = c.mp;
            d.mpSlots = c.slots;
        }
        d.mainLevel = m.valid ? m.levelDbfs + levelCalEff : NAN;
        for (int i = 0; i < carrierMon.count(); i++) {
            const auto& c = carrierMon.at(i);
            if (!c.valid) { continue; }
            d.carriers.push_back({ c.label, c.freqHz / 1e6, c.level + levelCalEff, c.sync ? c.mer : NAN, c.sync });
        }
        auto subs = subscribers.snapshot();
        for (auto& sub : subs) { if (sub.kind == TetraSubscriber::KIND_GROUP) { d.groups++; } else { d.stations++; } }
        d.traffic = (int)subscribers.trafficSnapshot().size();
        for (auto& e : meas.getEvents()) { d.events.push_back({ e.t, e.text }); }
        d.crcLimit = evalView.crcLimit;
        std::string dir = logFolder.expandString(logFolder.path);
        if (dir.empty()) { return "!No log folder (Measurements)"; }
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (!std::filesystem::is_directory(dir)) { return "!Log folder not valid (Measurements): " + dir; }
        return TetraReport::write(dir, d);
    }

    void drawReport(float width) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + width);
        ImGui::TextDisabled("%s", "One HTML file with the results of the session (summary, trends, timeslots, TX and burst analysis, multipath, carriers, events) in the log folder. Print it to PDF from the browser.");
        ImGui::PopTextWrapPos();
        tw::Text("Notes:");
        ImGui::SetNextItemWidth(width);
        if (ImGui::InputText(CONCAT("##_tetra_rep_notes_", name), reportNotes, sizeof(reportNotes))) {
            config.acquire();
            config.conf[name]["reportNotes"] = std::string(reportNotes);
            config.release(true);
        }
        tw::TextDisabled("Period: the time span chosen under Trends.");
        if (ImGui::Button(CONCAT("Create report##_tetra_rep_", name), ImVec2(width, 0))) {
            reportPath = makeReport();
            if (!reportPath.empty() && reportPath[0] != '!') { opener.open(reportPath); }
        }
        if (!reportPath.empty()) {
            if (reportPath[0] == '!') { tw::TextColored(tc::bad(), "%s", reportPath.c_str() + 1); }
            else {
                ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + width);
                ImGui::TextDisabled("%s", reportPath.c_str());
                ImGui::PopTextWrapPos();
                if (ImGui::SmallButton(CONCAT("Open##_tetra_rep_open_", name))) { opener.open(reportPath); }
                if (tw::SameSmallButton(CONCAT("Open folder##_tetra_rep_dir_", name))) {
                    opener.open(std::filesystem::path(reportPath).parent_path().string());
                }
                if (tw::SameSmallButton(CONCAT("Copy path##_tetra_rep_copy_", name))) { ImGui::SetClipboardText(reportPath.c_str()); }
            }
        }
        drawOpenerStatus(width);
    }

    void drawOpenerStatus(float width) {
        ExternalOpener::State os = opener.getState();
        if (os == ExternalOpener::RUNNING) { tw::TextDisabled("Opening ..."); }
        else if (os == ExternalOpener::FAILED) {
            ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + width);
            ImGui::TextColored(tc::bad(), "%s", (opener.message() + " - open the file by hand: Copy path, then paste it into the browser's address bar.").c_str());
            ImGui::PopTextWrapPos();
        }
    }

    // automation (documentation screenshots): one own window at a fixed place
    void placeSolo(const char* key = nullptr) {
        // layout preset: place the window at its tile for a few frames
        if (layoutFrames > 0 && key) {
            auto it = layoutRects.find(key);
            if (it != layoutRects.end()) {
                const ImGuiViewport* vp = ImGui::GetMainViewport();
                const float* r = it->second.r;
                ImGui::SetNextWindowPos(ImVec2(vp->WorkPos.x + r[0] * vp->WorkSize.x, vp->WorkPos.y + r[1] * vp->WorkSize.y), ImGuiCond_Always);
                ImGui::SetNextWindowSize(ImVec2(r[2] * vp->WorkSize.x, r[3] * vp->WorkSize.y), ImGuiCond_Always);
                return;
            }
        }
        if (soloFrames <= 0) { return; }
        ImGui::SetNextWindowPos(ImVec2(soloRect[0], soloRect[1]), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(soloRect[2], soloRect[3]), ImGuiCond_Always);
        ImGui::SetNextWindowFocus();
    }

    void floatingWindow(bool& detached, bool& focus, const char* title, const char* key, ImVec2 size, ImVec2 pos, void (TetraDemodulatorModule::*body)(float)) {
        if (!detached) { return; }
        ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(pos, ImGuiCond_FirstUseEver);
        placeSolo(key);
        bool open = true;
        std::string t = std::string(title) + " (" + name + ")##_tetra_" + key + "_" + name;
        if (focus) { ImGui::SetNextWindowFocus(); focus = false; }
        if (ImGui::Begin(t.c_str(), &open)) {
            (this->*body)(ImGui::GetContentRegionAvail().x);
            touch::dragScroll();
        }
        ImGui::End();
        if (!open) {
            detached = false;
            config.acquire();
            config.conf[name][std::string(key) + "Detached"] = false;
            config.release(true);
        }
    }

    void drawFloatingWindows() {
        if (!enabled) { return; }
        {
            std::lock_guard<std::mutex> lck(dbgMtx);
            if (soloReq >= 0) {
                bool* f[8] = { &constDetached, &subsDetached, &netDetached, &txaDetached, &trendDetached, &carrDetached, &burstDetached, &mpDetached };
                for (int i = 0; i < 8; i++) { *f[i] = (i == soloReq); }
                soloReq = -1;
                soloFrames = 30;
            }
        }
        if (soloFrames > 0) { soloFrames--; }
        floatingWindow(measDetached, focusMeasWin, "TETRA measurements", "meas", ImVec2(440, 560), ImVec2(500, 90), &TetraDemodulatorModule::drawMeasWindow);
        floatingWindow(gpsDetached, focusGpsWin, "TETRA GPS & coverage", "gps", ImVec2(460, 520), ImVec2(520, 120), &TetraDemodulatorModule::drawGpsCoverage);
        if (layoutFrames > 0) { layoutFrames--; }
        floatingWindow(trendDetached, focusTrendWin, "TETRA trends & timeslots", "trend", ImVec2(620, 640), ImVec2(560, 90), &TetraDemodulatorModule::drawTrends);
        floatingWindow(carrDetached, focusCarrWin, "TETRA carriers", "carr", ImVec2(560, 420), ImVec2(600, 130), &TetraDemodulatorModule::drawCarriers);
        floatingWindow(burstDetached, focusBurstWin, "TETRA burst analysis", "burst", ImVec2(520, 620), ImVec2(640, 110), &TetraDemodulatorModule::drawBursts);
        {
            bool was = mpDetached;
            floatingWindow(mpDetached, focusMpWin, "TETRA multipath", "mp", ImVec2(500, 420), ImVec2(680, 140), &TetraDemodulatorModule::drawMultipath);
            if (was && !mpDetached) { saveMpConfig(); }
        }
        if (txaDetached) {
            ImGui::SetNextWindowSize(ImVec2(560, 760), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowPos(ImVec2(520, 100), ImGuiCond_FirstUseEver);
            placeSolo("txa");
            bool open = true;
            std::string title = "TETRA TX analysis (" + name + ")##_tetra_twin_" + name;
            if (focusTxaWin) { ImGui::SetNextWindowFocus(); focusTxaWin = false; }
            if (ImGui::Begin(title.c_str(), &open)) {
                drawTxAnalysis(ImGui::GetContentRegionAvail().x);
                touch::dragScroll();
            }
            ImGui::End();
            if (!open) {
                txaDetached = false;
                config.acquire();
                config.conf[name]["txaDetached"] = false;
                config.release(true);
            }
        }
        if (constDetached) {
            ImGui::SetNextWindowSize(ImVec2(420, 720), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowPos(ImVec2(470, 80), ImGuiCond_FirstUseEver);
            placeSolo("const");
            bool open = true;
            std::string title = "TETRA constellation (" + name + ")##_tetra_cwin_" + name;
            if (focusConstWin) { ImGui::SetNextWindowFocus(); focusConstWin = false; }
            if (ImGui::Begin(title.c_str(), &open)) {
                float w = ImGui::GetContentRegionAvail().x;
                drawConstellationOptions("w");
                drawConstellation(w);
                touch::dragScroll();
            }
            ImGui::End();
            if (!open) {
                constDetached = false;
                config.acquire();
                config.conf[name]["constDetached"] = false;
                config.release(true);
            }
        }
        if (netDetached && decoder_mode == 0) {
            ImGui::SetNextWindowSize(ImVec2(400, 560), ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowPos(ImVec2(470, 120), ImGuiCond_FirstUseEver);
            placeSolo("net");
            bool open = true;
            std::string title = "TETRA network & cell (" + name + ")##_tetra_nwin_" + name;
            if (focusNetWin) { ImGui::SetNextWindowFocus(); focusNetWin = false; }
            if (ImGui::Begin(title.c_str(), &open)) {
                drawNetworkBody(ImGui::GetContentRegionAvail().x);
                touch::dragScroll();
            }
            ImGui::End();
            if (!open) {
                netDetached = false;
                config.acquire();
                config.conf[name]["netDetached"] = false;
                config.release(true);
            }
        }
        if (subsDetached && hamMode) {
            ImGui::SetNextWindowSize(ImVec2(560, 360), ImGuiCond_FirstUseEver);
            // not on top of the constellation window
            ImGui::SetNextWindowPos(ImVec2(900, 80), ImGuiCond_FirstUseEver);
            placeSolo("subs");
            bool open = true;
            std::string title = "TETRA stations & traffic (" + name + ")##_tetra_swin_" + name;
            if (focusSubsWin) { ImGui::SetNextWindowFocus(); focusSubsWin = false; }
            if (ImGui::Begin(title.c_str(), &open)) {
                drawSubscribersBody(ImGui::GetContentRegionAvail().x, 0);
            }
            ImGui::End();
            if (!open) {
                subsDetached = false;
                config.acquire();
                config.conf[name]["subsDetached"] = false;
                config.release(true);
            }
        }
    }

    // ---------------- Subscribers (amateur radio mode) ----------------

    void drawSubscribers(float menuWidth) {
        if (inTiles(S_SUBS)) {
            if (sectionHeader(S_SUBS, (sectionLabel(S_SUBS) + name).c_str(), ImGuiTreeNodeFlags_DefaultOpen)) { tw::TextDisabled("Shown in a tile."); }
            return;
        }
        if (!sectionHeader(S_SUBS, CONCAT("Stations & Traffic##_tetra_subs_", name), ImGuiTreeNodeFlags_DefaultOpen)) { return; }
        if (subsDetached) {
            tw::TextDisabled("Shown in its own window.");
            if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_sfront_", name))) { focusSubsWin = true; }
            return;
        }
        drawSubscribersBody(menuWidth, 12);
    }

    // Name for an SSI: groups from the Brandmeister talkgroup list, individuals
    // (7 digits users, 6 digits repeaters) from radioid.net
    void describeSsi(uint32_t ssi, int kind, std::string& call, std::string& info) {
        call.clear();
        info.clear();
        if (ssi == 0xFFFFFF) { call = "ALL"; info = "broadcast address (all stations)"; return; }
        RadioIdEntry e;
        std::string tg;
        if (kind == TetraSubscriber::KIND_GROUP) {
            if (radioid.lookupGroup(ssi, tg)) { call = "TG"; info = tg; }
            return;
        }
        if (radioid.lookup(ssi, e)) { call = e.callsign; info = e.info; return; }
        if (kind == TetraSubscriber::KIND_UNKNOWN && radioid.lookupGroup(ssi, tg)) { call = "TG?"; info = tg; }
    }

    static const char* kindName(int k) {
        switch (k) {
        case TetraSubscriber::KIND_INDIVIDUAL: return "ISSI";
        case TetraSubscriber::KIND_GROUP: return "GSSI";
        case TetraSubscriber::KIND_BROADCAST: return "BC";
        default: return "?";
        }
    }

    static void agoText(long ago, char* buf, size_t n) {
        if (ago < 60) { snprintf(buf, n, "%lds", ago); }
        else if (ago < 3600) { snprintf(buf, n, "%ldm", ago / 60); }
        else { snprintf(buf, n, "%ldh", ago / 3600); }
    }

    void drawSubscribersBody(float menuWidth, int visibleRows) {
        // Name databases
        tw::TextDisabled("%s", radioid.status().c_str());
        bool busy = radioid.busy();
        if (busy) { style::beginDisabled(); }
        float w2 = menuWidth / 2.0f - 4;
        if (ImGui::Button(CONCAT("Download name lists##_tetra_rid_dl_", name), ImVec2(w2, 0))) {
            radioid.downloadAsync();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("radioid.net users (7-digit ISSI) and repeaters (6-digit),\nBrandmeister talkgroup names for groups (GSSI).");
        }
        if (tw::SameButton(CONCAT("Reload files##_tetra_rid_rl_", name), ImVec2(w2, 0))) {
            radioid.loadAsync();
        }
        if (busy) { style::endDisabled(); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Files: %s\n       %s\n       %s", radioid.usersPath().c_str(), radioid.rptrsPath().c_str(), radioid.tgPath().c_str());
        }

        std::vector<TetraSubscriber> list = subscribers.snapshot();
        time_t now = time(nullptr);
        std::vector<TetraSubscriber> stations, groups;
        for (auto& sub : list) {
            if (sub.kind == TetraSubscriber::KIND_GROUP) { groups.push_back(sub); }
            else { stations.push_back(sub); }
        }

        auto tableHeight = [&](int rows) {
            return visibleRows > 0 ? ImGui::GetTextLineHeightWithSpacing() * ((std::min<int>)(rows, visibleRows) + 1.5f)
                                   : (std::max)(ImGui::GetContentRegionAvail().y - ImGui::GetTextLineHeightWithSpacing() * 1.5f, 60.0f);
        };
        // Tables: one line per row (cells clip with "..."), columns sized to
        // their content, resizable, movable (drag the header) and hideable
        // (right-click the header). If the window is narrower than the minimum
        // widths, the table scrolls sideways; the first column stays in place.
        ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY | ImGuiTableFlags_ScrollX |
                                ImGuiTableFlags_SizingFixedFit | ImGuiTableFlags_Resizable | ImGuiTableFlags_Reorderable | ImGuiTableFlags_Hideable;
        auto colW = [](const char* sample) { return ImGui::CalcTextSize(sample).x + ImGui::GetStyle().CellPadding.x * 2.0f; };
        auto innerW = [&](float minTotal) { return (std::max)(menuWidth, minTotal); };
        tw::CellScope cells;
        ImVec4 green = tc::good(), yellow = tc::val(), grey = tc::grey();

        if (!ImGui::BeginTabBar(CONCAT("##_tetra_st_tabs_", name))) { return; }

        // ---- Stations ----
        std::string stTitle = "Stations (" + std::to_string(stations.size()) + ")###_tetra_tab_st_" + name;
        if (ImGui::BeginTabItem(stTitle.c_str())) {
            if (ImGui::SmallButton(CONCAT("Clear all##_tetra_subs_clr_", name))) { subscribers.clear(); stations.clear(); }
            float minSt = colW("16777215") + colW("ISSI") + colW("DB0XXX-12") + colW("4CE") + colW("59m") + 3 * colW("talking in 2620001");
            if (ImGui::BeginTable(CONCAT("##_tetra_subs_tab_", name), 8, flags, ImVec2(menuWidth, tableHeight((int)stations.size())), innerW(minSt))) {
                ImGui::TableSetupScrollFreeze(1, 1);
                ImGui::TableSetupColumn("ISSI", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoHide, colW("16777215"), 0);
                ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, colW("ISSI"), 1);
                ImGui::TableSetupColumn("Call", ImGuiTableColumnFlags_WidthFixed, colW("DB0XXX-12"), 2);
                ImGui::TableSetupColumn("TS", ImGuiTableColumnFlags_WidthFixed, colW("4CE"), 3);
                ImGui::TableSetupColumn("Seen", ImGuiTableColumnFlags_WidthFixed, colW("59m"), 4);
                ImGui::TableSetupColumn("Groups (scan list)", ImGuiTableColumnFlags_WidthStretch, 1.0f, 6);
                ImGui::TableSetupColumn("Activity", ImGuiTableColumnFlags_WidthStretch, 1.2f, 5);
                ImGui::TableSetupColumn("Info", ImGuiTableColumnFlags_WidthStretch, 1.0f, 7);
                ImGui::TableHeadersRow();
                for (auto& sub : stations) {
                    std::string call, info;
                    describeSsi(sub.ssi, sub.kind, call, info);
                    long ago = (long)(now - sub.lastSeen);
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    tw::TextColored(sub.talkingIn ? green : (ago < 120 ? yellow : grey), "%u", sub.ssi);
                    ImGui::TableSetColumnIndex(1);
                    tw::TextDisabled("%s", kindName(sub.kind));
                    if (ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("ISSI = individual (seen as calling/talking party or registering)\n"
                                          "? = only seen as MAC address so far (individual or group)");
                    }
                    ImGui::TableSetColumnIndex(2);
                    if (!call.empty()) { tw::TextU(call.c_str()); } else { tw::TextDisabled("-"); }
                    ImGui::TableSetColumnIndex(3);
                    tw::Text("%u%s%s", sub.lastTn, sub.chanAlloc ? "C" : "", sub.encrypted ? "E" : "");
                    ImGui::TableSetColumnIndex(4);
                    char ab[32];
                    agoText(ago, ab, sizeof(ab));
                    tw::TextU(ab);
                    ImGui::TableSetColumnIndex(6);   // Activity (shown after Groups)
                    if (sub.talkingIn) {
                        std::string gc, gi;
                        describeSsi(sub.talkingIn, TetraSubscriber::KIND_GROUP, gc, gi);
                        tw::TextColored(green, "TALKING in %u%s", sub.talkingIn, gi.empty() ? "" : (" " + gi).c_str());
                    }
                    else if (!sub.lastActivity.empty()) { tw::TextU(sub.lastActivity.c_str()); }
                    if (sub.hasPos && ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Last position %.5f, %.5f", sub.lat, sub.lon);
                    }
                    ImGui::TableSetColumnIndex(5);   // Groups (scan list)
                    if (!sub.groups.empty()) {
                        std::string gl;
                        for (size_t gi = 0; gi < sub.groups.size(); gi++) { gl += (gi ? ", " : "") + std::to_string(sub.groups[gi]); }
                        tw::TextU(gl.c_str());
                        if (ImGui::IsItemHovered()) {
                            std::string tip = "Attached talkgroups (from group attach/detach / registration):";
                            for (auto g : sub.groups) {
                                std::string c, i;
                                describeSsi(g, TetraSubscriber::KIND_GROUP, c, i);
                                tip += "\n" + std::to_string(g) + (i.empty() ? "" : "  " + i);
                            }
                            ImGui::SetTooltip("%s", tip.c_str());
                        }
                    }
                    ImGui::TableSetColumnIndex(7);
                    tw::TextU(info.c_str());
                }
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }

        // ---- Groups ----
        std::string grTitle = "Groups (" + std::to_string(groups.size()) + ")###_tetra_tab_gr_" + name;
        if (ImGui::BeginTabItem(grTitle.c_str())) {
            float minGr = colW("16777215") + colW("TETRA Oberbayern") + colW("4E") + colW("59m") + 2 * colW("2620001 DB0XXX") + colW("Attached");
            if (ImGui::BeginTable(CONCAT("##_tetra_grp_tab_", name), 7, flags, ImVec2(menuWidth, tableHeight((int)groups.size())), innerW(minGr))) {
                ImGui::TableSetupScrollFreeze(1, 1);
                ImGui::TableSetupColumn("GSSI", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoHide, colW("16777215"));
                ImGui::TableSetupColumn("Talkgroup", ImGuiTableColumnFlags_WidthStretch, 1.2f);
                ImGui::TableSetupColumn("TS", ImGuiTableColumnFlags_WidthFixed, colW("4E"));
                ImGui::TableSetupColumn("Seen", ImGuiTableColumnFlags_WidthFixed, colW("59m"));
                ImGui::TableSetupColumn("Talker", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupColumn("Last talker", ImGuiTableColumnFlags_WidthStretch, 1.0f);
                ImGui::TableSetupColumn("Attached", ImGuiTableColumnFlags_WidthFixed, colW("Attached"));
                ImGui::TableHeadersRow();
                for (auto& g : groups) {
                    std::string call, info;
                    describeSsi(g.ssi, TetraSubscriber::KIND_GROUP, call, info);
                    long ago = (long)(now - g.lastSeen);
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    tw::TextColored(g.talker ? green : (ago < 120 ? yellow : grey), "%u", g.ssi);
                    ImGui::TableSetColumnIndex(1);
                    if (!info.empty()) { tw::TextU(info.c_str()); } else { tw::TextDisabled("-"); }
                    ImGui::TableSetColumnIndex(2);
                    tw::Text("%u%s", g.lastTn, g.encrypted ? "E" : "");
                    ImGui::TableSetColumnIndex(3);
                    char ab[32];
                    agoText(ago, ab, sizeof(ab));
                    tw::TextU(ab);
                    ImGui::TableSetColumnIndex(4);
                    if (g.talker) {
                        std::string tc, ti;
                        describeSsi(g.talker, TetraSubscriber::KIND_INDIVIDUAL, tc, ti);
                        tw::TextColored(green, "%u %s", g.talker, tc.c_str());
                    }
                    ImGui::TableSetColumnIndex(5);
                    if (g.lastTalker) {
                        std::string tc, ti;
                        describeSsi(g.lastTalker, TetraSubscriber::KIND_INDIVIDUAL, tc, ti);
                        tw::Text("%u %s", g.lastTalker, tc.c_str());
                    }
                    ImGui::TableSetColumnIndex(6);
                    {
                        int members = 0;
                        std::string who;
                        for (auto& st : stations) {
                            if (std::find(st.groups.begin(), st.groups.end(), g.ssi) != st.groups.end()) {
                                members++;
                                std::string c, i;
                                describeSsi(st.ssi, TetraSubscriber::KIND_INDIVIDUAL, c, i);
                                who += (who.empty() ? "" : "\n") + std::to_string(st.ssi) + " " + c;
                            }
                        }
                        if (members) {
                            tw::Text("%d", members);
                            if (ImGui::IsItemHovered()) { ImGui::SetTooltip("%s", who.c_str()); }
                        } else { tw::TextDisabled("-"); }
                    }
                }
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }

        // ---- Traffic ----
        std::vector<TetraTrafficEntry> traffic = subscribers.trafficSnapshot();
        std::string trTitle = "Traffic (" + std::to_string(traffic.size()) + ")###_tetra_tab_tr_" + name;
        if (ImGui::BeginTabItem(trTitle.c_str())) {
            if (ImGui::SmallButton(CONCAT("Clear##_tetra_tr_clr_", name))) { subscribers.clearTraffic(); traffic.clear(); }
            tw::SameCheckbox(CONCAT("Hide call control##_tetra_tr_cc_", name), &trafficHideControl);
            float minTr = colW("00:00:00") + colW("TS") + colW("D-TX-GRANTED") + 2 * colW("2620001 DB0XXX") + colW("SDS text: Hello World");
            if (ImGui::BeginTable(CONCAT("##_tetra_tr_tab2_", name), 6, flags, ImVec2(menuWidth, tableHeight((int)traffic.size())), innerW(minTr))) {
                ImGui::TableSetupScrollFreeze(1, 1);
                ImGui::TableSetupColumn("Time", ImGuiTableColumnFlags_WidthFixed | ImGuiTableColumnFlags_NoHide, colW("00:00:00"));
                ImGui::TableSetupColumn("To", ImGuiTableColumnFlags_WidthStretch, 0.8f);
                ImGui::TableSetupColumn("PDU", ImGuiTableColumnFlags_WidthFixed, colW("D-TX-GRANTED"));
                ImGui::TableSetupColumn("From", ImGuiTableColumnFlags_WidthStretch, 0.8f);
                ImGui::TableSetupColumn("TS", ImGuiTableColumnFlags_WidthFixed, colW("TS"));
                ImGui::TableSetupColumn("Content", ImGuiTableColumnFlags_WidthStretch, 2.0f);
                ImGui::TableHeadersRow();
                // newest first; only the visible rows are drawn (up to 1000 entries)
                std::vector<const TetraTrafficEntry*> rows;
                rows.reserve(traffic.size());
                for (auto it = traffic.rbegin(); it != traffic.rend(); ++it) {
                    bool control = it->pdu != "D-SDS-DATA" && it->pdu != "D-STATUS";
                    if (trafficHideControl && control) { continue; }
                    rows.push_back(&*it);
                }
                ImGuiListClipper clipper;
                clipper.Begin((int)rows.size());
                while (clipper.Step()) for (int ri = clipper.DisplayStart; ri < clipper.DisplayEnd; ri++) {
                    const TetraTrafficEntry* it = rows[ri];
                    bool control = it->pdu != "D-SDS-DATA" && it->pdu != "D-STATUS";
                    struct tm lt;
#ifdef _WIN32
                    localtime_s(&lt, &it->t);
#else
                    localtime_r(&it->t, &lt);
#endif
                    char ts[16];
                    strftime(ts, sizeof(ts), "%H:%M:%S", &lt);
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    tw::TextDisabled("%s", ts);
                    ImGui::TableSetColumnIndex(4);
                    tw::Text("%u", it->tn);
                    ImGui::TableSetColumnIndex(2);
                    if (control) { tw::TextU(it->pdu.c_str()); } else { tw::TextColored(yellow, "%s", it->pdu.c_str()); }
                    ImGui::TableSetColumnIndex(3);
                    if (it->from) {
                        std::string c, i;
                        describeSsi(it->from, TetraSubscriber::KIND_INDIVIDUAL, c, i);
                        tw::Text("%u %s", it->from, c.c_str());
                    }
                    ImGui::TableSetColumnIndex(1);
                    if (it->to) {
                        std::string c, i;
                        describeSsi(it->to, it->toGroup ? TetraSubscriber::KIND_GROUP : TetraSubscriber::KIND_UNKNOWN, c, i);
                        tw::Text("%s%u %s", it->toGroup ? "G " : "", it->to, it->toGroup ? i.c_str() : c.c_str());
                    }
                    ImGui::TableSetColumnIndex(5);
                    tw::TextU(it->text.c_str());
                    if (it->hasPos && ImGui::IsItemHovered()) {
                        ImGui::SetTooltip("Position %.5f, %.5f", it->lat, it->lon);
                    }
                }
                ImGui::EndTable();
            }
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    static void _constDiagSinkHandler(dsp::complex_t* data, int count, void* ctx) {
        TetraDemodulatorModule* _this = (TetraDemodulatorModule*)ctx;
        // The Costas loop derotates every symbol by -pi/4 (cumulative), which
        // folds the 8 pi/4-DQPSK points onto 4. For the 8-point view, re-apply
        // a +k*pi/4 rotation per symbol. Any constant offset is a multiple of
        // pi/4, so the 8 target positions are always the same.
        static const dsp::complex_t rot8[8] = {
            { 1.0f, 0.0f }, { 0.70710678f, 0.70710678f }, { 0.0f, 1.0f }, { -0.70710678f, 0.70710678f },
            { -1.0f, 0.0f }, { -0.70710678f, -0.70710678f }, { 0.0f, -1.0f }, { 0.70710678f, -0.70710678f }
        };
        dsp::complex_t tmp[TetraConstellationView::N];
        int n = (std::min)(count, (int)TetraConstellationView::N);
        // raw pi/4-DQPSK frame (8 points)
        for (int i = 0; i < n; i++) {
            tmp[i] = data[i] * rot8[(_this->rotIdx + i) & 7];
        }
        if (_this->compOriginOffset && n > 0) {
            // Origin (I/Q) offset is a constant vector in the raw frame: remove
            // the block mean, like the "compensate origin offset" option of
            // vector signal analysers. Display only - measurements stay raw.
            double mr = 0.0, mi = 0.0;
            for (int i = 0; i < n; i++) { mr += tmp[i].re; mi += tmp[i].im; }
            mr /= n; mi /= n;
            for (int i = 0; i < n; i++) { tmp[i].re -= (float)mr; tmp[i].im -= (float)mi; }
        }
        if (!_this->constellation8) {
            // back to the 4-point (derotated) frame
            for (int i = 0; i < n; i++) {
                const dsp::complex_t& r = rot8[(_this->rotIdx + i) & 7];
                dsp::complex_t c = { r.re, -r.im };
                tmp[i] = tmp[i] * c;
            }
        }
        _this->constView.set(tmp, n);
        _this->rotIdx = (_this->rotIdx + count) & 7;
    }

    static void _demodSinkHandler(uint8_t* data, int count, void* ctx) {
        TetraDemodulatorModule* _this = (TetraDemodulatorModule*)ctx;
        if(_this->conn && _this->conn->isOpen()) {
            _this->conn->send(data, count);
        }
        for(int j = 0; j < count; j++) {
            for(int i = 0; i < 44; i++) {
                _this->tsfind_buffer[i] = _this->tsfind_buffer[i+1];
            }
            _this->tsfind_buffer[44] = data[j];
            if(!memcmp(_this->tsfind_buffer, training_seq_n, sizeof(training_seq_n)) ||
                !memcmp(_this->tsfind_buffer, training_seq_p, sizeof(training_seq_p)) ||
                !memcmp(_this->tsfind_buffer, training_seq_q, sizeof(training_seq_q)) ||
                !memcmp(_this->tsfind_buffer, training_seq_N, sizeof(training_seq_N)) ||
                !memcmp(_this->tsfind_buffer, training_seq_P, sizeof(training_seq_P)) ||
                !memcmp(_this->tsfind_buffer, training_seq_x, sizeof(training_seq_x)) ||
                !memcmp(_this->tsfind_buffer, training_seq_X, sizeof(training_seq_X)) ||
                !memcmp(_this->tsfind_buffer, training_seq_y, sizeof(training_seq_y))
            ) {
                _this->tsfound = true;
                _this->symsbeforeexpire = 2048;
            }
            if(_this->symsbeforeexpire > 0) {
                _this->symsbeforeexpire--;
                if(_this->symsbeforeexpire == 0) {
                    _this->tsfound = false;
                }
            }
        }
    }

    static void sampleRateChangeHandler(float sampleRate, void* ctx) {
        TetraDemodulatorModule* _this = (TetraDemodulatorModule*)ctx;
        _this->audioSampleRate = sampleRate;
        _this->resamp.stop();
        _this->resamp.setOutSamplerate(_this->audioSampleRate);
        _this->resamp.start();
    }

    std::string name;
    bool enabled = true;

    VFOManager::VFO* vfo;

    dsp::demod::PI4DQPSK mainDemodulator;
    dsp::routing::Splitter<dsp::complex_t> constDiagSplitter;
    dsp::stream<dsp::complex_t> constDiagStream;
    dsp::buffer::Reshaper<dsp::complex_t> constDiagReshaper;
    dsp::sink::Handler<dsp::complex_t> constDiagSink;
    TetraConstellationView constView;
    bool constDetached = false;
    bool compOriginOffset = true;    // edition default: display without I/Q offset
    bool afcEnabled = true;     // edition default
    bool afcAnchorValid = false;
    double afcAnchor = 0.0, afcLastSet = 0.0;
    std::string afcStatus;
    std::chrono::steady_clock::time_point afcLastStep;
    bool autoGain = true;       // edition default
    // 0 level once and hold, 1 follow continuously, 2 automatic: follow while
    // moving (GPS, > 5 km/h), hold when standing (30 s) or without GPS
    int gainMode = 2;
    bool gainHolding = false;   // levelled, gain held
    int gainQuiet = 0;          // evaluations without a step (levelling done after 2)
    float gainHoldSnr = NAN;    // band SNR when the gain was fixed
    std::string gainHoldWarn;   // why re-levelling is advised
    std::string gainHeldAt;     // wall clock time of the levelling
    bool gainMoving = false;
    double gainSpeedAvg = 0.0;
    int gainStillS = 0;
    std::chrono::steady_clock::time_point gainMoveLast;
    std::string gainStatus;
    std::chrono::steady_clock::time_point gainLastStep;
    int gainLastSet = -1;
    SourceGainInterface* ovIf = nullptr;
    long long ovLast = -1;
    bool lossThisSecond = false;               // overflow or timing slip since the last alarm check
    uint32_t slipLast = 0;
    std::vector<int> secOrder;                 // menu sections in display order
    int secMoveSrc = -1, secMoveDst = -1;
    bool secMoveAfter = false, secResetReq = false;
    // auto RF gain by SNR: state of the last step and an upper limit found
    bool agcLastUp = false;
    double agcPrevSnr = NAN, agcPrevGainDb = NAN, agcCeilDb = NAN;
    std::chrono::steady_clock::time_point agcCeilT;
    TetraBandMonitor bandMon;
    TetraBandMonitor::Result bandRes;
    std::chrono::steady_clock::time_point bandLast;
    SourceGainInterface* gainLastIf = nullptr;       // gain step auto RF gain last saw / set (detects manual changes)
    bool subsDetached = false;
    bool trafficHideControl = false;
    bool focusSubsWin = false;
    bool netDetached = false;
    bool focusNetWin = false;
    TetraAnalysisTap anaTap;   // declared before its users: destroyed after them
    TetraTxAnalyzer txa;
    bool txaRun = false;
    bool txaDetached = false;
    bool focusTxaWin = false;
    int txaFftIdx = 1;        // 0: 4096, 1: 8192, 2: 16384
    int txaAvg = 20;
    float txaLimit[3] = { -60.0f, -70.0f, -70.0f };   // dBc at 25 / 50 / 75 kHz
    int freqRef = 0;   // frequency reference of the SDR: 0 internal, 1 external 10 MHz, 2 calibrated
    bool focusConstWin = false;
    // trends, further carriers, burst analysis (v0.18)
    TetraTrendView trendView;
    FileSelect trendFile { "%ROOT%/tetra_logs", { "TETRA CSV logs", "*.csv" } };
    std::string trendFileError;
    std::mutex trendReqMtx;
    std::string trendLoadReq;   // debug: open this CSV log in the GUI thread
    int trendTabReq = -1;       // debug: select this trends tab
    bool trendDetached = false, focusTrendWin = false;
    bool measDetached = false, focusMeasWin = false;
    bool gpsDetached = false, focusGpsWin = false;
    struct LayoutRect { float r[4]; };
    std::map<std::string, LayoutRect> layoutRects;   // tiles of the active layout preset (fractions of the work area)
    int layoutFrames = 0;
    TetraCarrierMonitor carrierMon;
    bool carrDetached = false, focusCarrWin = false;
    TetraBurstAnalyzer burstAn;
    TetraBurstSettings burstSt;
    bool burstRun = false;
    bool burstDetached = false, focusBurstWin = false;
    std::atomic<double> pendingCarrierHz { 0.0 };
    TetraChannelAnalyzer chanAn;
    TetraVemAnalyzer vemAn;
    bool vemRun = true;
    bool chanRaster = true;   // 25 kHz channel boundaries around the TETRA channel in the spectrum
    bool vfoLocked = false;   // no mouse tuning of the TETRA VFO (drag pans the view)
    bool snapCarrier = true;  // a mouse tune lands on the centre of the nearest carrier
    bool snapMouseDown = false, snapPending = false;
    double snapStartFreq = 0.0, snapClickOffset = 0.0;
    std::chrono::steady_clock::time_point snapReqTime;
    std::string snapStatus;
    bool startViewDone = false;
    EventHandler<ImGui::WaterFall::FFTRedrawArgs> fftRedrawHandler;
    bool mpRun = false, mpDetached = false, focusMpWin = false;
    int mpMem = 1;
    TetraEvalView evalView;
    std::string evalFile;
    TetraAlarmSettings alarmSt;
    TetraAlarms alarms;
    time_t lastAlarmT = 0;
    char reportNotes[256] = "";
    std::string reportPath;
    std::atomic<bool> reportReq { false };
    std::mutex reportMtx;
    std::string reportResult;
    std::string burstStatus;
    std::mutex dbgMtx;
    int dbgCarr[3] = { 0, 0, 0 };
    int dbgAlarmTotal = 0;
    int soloReq = -1, soloFrames = 0;
    float soloRect[4] = { 380, 40, 820, 760 };
    bool dbgAlarmActive = false;
    EventHandler<ImGuiContext*> windowDrawHandler;

    dsp::stream<dsp::complex_t> demodStream;

    dsp::DQPSKSymbolExtractor symbolExtractor;
    dsp::BitUnpacker bitsUnpacker;

    dsp::sink::Handler<uint8_t> demodSink;

    dsp::osmotetradec osmotetradecoder;

    EventHandler<float> srChangeHandler;
    dsp::AudioDecoupler audioDecoupler;
    dsp::multirate::RationalResampler<float> resamp;
    dsp::convert::MonoToStereo outconv;
    SinkManager::Stream stream;
    double audioSampleRate = 48000.0;


    int decoder_mode = 0;

    bool constellation8 = true;
    bool hamMode = true; // amateur radio edition: default
    int hamTable[8][16];
    std::string hamTableStatus;
    TetraSubscriberTable subscribers;
    NmeaGps gps;
    std::string gpsPort;
    int gpsBaud = 9600;
    std::vector<std::string> gpsPorts;
    CoverageMapExporter mapExport;
    int mapMetric = 0;
    std::vector<TileCfg> tiles;   // tile column right of the waterfall, top to bottom
    float tileColW = 440.0f;      // px (saved without the UI scale)
    bool tileBottom = false;      // Stations & Traffic as a strip below the waterfall
    float tileBottomH = 220.0f;
    int tilePreset = 0;
    int tilesMigrate = 0;
    bool tilesDirty = false, sideRegistered = false, bottomRegistered = false;
    int touchTab = 0, touchStepIdx = 2;
    std::atomic<int> layoutReq { -1 };
    TetraPropagationView propView;
    RadioIdDb radioid;
    float levelCalDb = 0.0f;
    float levelCalGainDb = NAN;   // RF gain (dB) when the calibration was entered
    float levelCalEff = 0.0f;     // calibration corrected by the gain change since then
    FolderSelect logFolder;
    ExternalOpener opener;
    std::string logError;
    TetraMeasurements meas;
    int rotIdx = 0;
    int duplexEditTable = 0;


    //Sequences from osmo-tetra-sq5bpf source
    /* 9.4.4.3.2 Normal Training Sequence */
    static const constexpr uint8_t training_seq_n[22] = { 1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,1, 0,0 };
    static const constexpr uint8_t training_seq_p[22] = { 0,1, 1,1, 1,0, 1,0, 0,1, 0,0, 0,0, 1,1, 0,1, 1,1, 1,0 };
    static const constexpr uint8_t training_seq_q[22] = { 1,0, 1,1, 0,1, 1,1, 0,0, 0,0, 0,1, 1,0, 1,0, 1,1, 0,1 };
    static const constexpr uint8_t training_seq_N[33] = { 1,1,1, 0,0,1, 1,0,1, 1,1,1, 0,0,0, 1,1,1, 1,0,0, 0,1,1, 1,1,0, 0,0,0, 0,0,0 };
    static const constexpr uint8_t training_seq_P[33] = { 1,0,1, 0,1,1, 1,1,1, 1,0,1, 0,1,0, 1,0,1, 1,1,0, 0,0,1, 1,0,0, 0,1,0, 0,1,0 };

    /* 9.4.4.3.3 Extended training sequence */
    static const constexpr uint8_t training_seq_x[30] = { 1,0, 0,1, 1,1, 0,1, 0,0, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,1, 0,0, 0,0, 1,1 };
    static const constexpr uint8_t training_seq_X[45] = { 0,1,1,1,0,0,1,1,0,1,0,0,0,0,1,0,0,0,1,1,1,0,1,1,0,1,0,1,0,1,1,1,1,1,0,1,0,0,0,0,0,1,1,1,0 };

    /* 9.4.4.3.4 Synchronization training sequence */
    static const constexpr uint8_t training_seq_y[38] = { 1,1, 0,0, 0,0, 0,1, 1,0, 0,1, 1,1, 0,0, 1,1, 1,0, 1,0, 0,1, 1,1, 0,0, 0,0, 0,1, 1,0, 0,1, 1,1 };
    uint8_t tsfind_buffer[45];
    bool tsfound = false;
    int symsbeforeexpire = 0;

    char hostname[1024];
    int port = 8355;

    std::shared_ptr<net::Socket> conn;

};

MOD_EXPORT void _INIT_() {
    std::string root = (std::string)core::args["root"];
    json def = json({});
    config.setPath(root + "/tetra_demodulator_config.json");
    config.load(def);
    config.enableAutoSave();
}

MOD_EXPORT ModuleManager::Instance* _CREATE_INSTANCE_(std::string name) {
    return new TetraDemodulatorModule(name);
}

MOD_EXPORT void _DELETE_INSTANCE_(void* instance) {
    delete (TetraDemodulatorModule*)instance;
}

MOD_EXPORT void _END_() {
    config.disableAutoSave();
    config.save();
}
