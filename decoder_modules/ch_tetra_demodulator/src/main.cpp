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
#include "tx_analyzer.h"
#include "trend_view.h"
#include "carrier_monitor.h"
#include "burst_analyzer.h"
#include "channel_analyzer.h"
#include "eval_view.h"
#include "alarms.h"
#include "report.h"
#include <version.h>
#include "measurements.h"
#include "subscribers.h"
#include "radioid.h"
#include "gps.h"
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
    /* Version:         */ 0, 19, 0,
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
        snprintf(hostname, sizeof(hostname), "%s", std::string(config.conf[name]["hostname"]).c_str());
        port = config.conf[name]["port"];
        bool startNow = config.conf[name]["sending"];
        if (config.conf[name].contains("levelCalDb")) {
            levelCalDb = config.conf[name]["levelCalDb"];
        }
        if (config.conf[name].contains("logPath")) {
            logFolder.setPath(config.conf[name]["logPath"]);
        }
        if (config.conf[name].contains("constellation8")) {
            constellation8 = config.conf[name]["constellation8"];
        }
        if (config.conf[name].contains("constDetached")) { constDetached = config.conf[name]["constDetached"]; }
        if (config.conf[name].contains("subsDetached")) { subsDetached = config.conf[name]["subsDetached"]; }
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
        if (config.conf[name].contains("trendDetached")) { trendDetached = config.conf[name]["trendDetached"]; }
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
        if (config.conf[name].contains("mp") && config.conf[name]["mp"].is_object()) {
            json& mp = config.conf[name]["mp"];
            if (mp.contains("run")) { mpRun = mp["run"]; }
            if (mp.contains("mem")) { mpMem = std::clamp((int)mp["mem"], 0, 3); }
            if (mp.contains("detached")) { mpDetached = mp["detached"]; }
        }
        if (config.conf[name].contains("crcLimit")) { evalView.crcLimit = config.conf[name]["crcLimit"]; }
        if (config.conf[name].contains("burst") && config.conf[name]["burst"].is_object()) {
            json& b = config.conf[name]["burst"];
            if (b.contains("timing")) { burstSt.timing = b["timing"]; }
            if (b.contains("transients")) { burstSt.transients = b["transients"]; }
            if (b.contains("floorSpan")) { burstSt.floorSpan = b["floorSpan"]; }
        }
        if (config.conf[name].contains("alarms") && config.conf[name]["alarms"].is_object()) {
            json& a = config.conf[name]["alarms"];
            auto gb = [&](const char* k, bool& v) { if (a.contains(k) && a[k].is_boolean()) { v = a[k]; } };
            auto gf = [&](const char* k, float& v) { if (a.contains(k) && a[k].is_number()) { v = a[k]; } };
            gb("enabled", alarmSt.enabled); gb("levelOn", alarmSt.levelOn); gf("levelBelow", alarmSt.levelBelow);
            gb("merOn", alarmSt.merOn); gf("merBelow", alarmSt.merBelow); gb("crcOn", alarmSt.crcOn); gf("crcAbove", alarmSt.crcAbove);
            gb("syncOn", alarmSt.syncOn); gb("ulOn", alarmSt.ulOn); gf("ulAbove", alarmSt.ulAbove);
            gb("beep", alarmSt.beep); gb("autoLog", alarmSt.autoLog);
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
            chanAn.setName(name + " channel");
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
        meas.stop();
        txa.stop();
        carrierMon.stopAll();
        carrierMon.stopLog();
        burstAn.stop();
        chanAn.stop();
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

        // Stop the measurement thread first: it reads the blocks and the VFO
        meas.stop();
        txa.stop();
        carrierMon.stopAll();
        carrierMon.stopLog();
        burstAn.stop();
        chanAn.stop();

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
            int alarmTotal;
            bool alarmActive;
            {
                std::lock_guard<std::mutex> lck(dbgMtx);
                alarmTotal = dbgAlarmTotal;
                alarmActive = dbgAlarmActive;
            }
            char b[2600];
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
                     "\"alarms_total\": %d, \"alarm_active\": %s}",
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
                     alarmTotal, alarmActive ? "true" : "false");
            return b;
        }
        return "{\"error\": \"unknown command: " + cmd + "\"}";
    }

private:

    void startMeasurements() {
        meas.setGps(&gps);
        meas.setLevelCal(levelCalDb);
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
        tw::Text("Level:"); tw::SameNext();
        if (m.valid) {
            if (_this->levelCalDb != 0.0f) {
                tw::TextColored(val, "%6.1f dBm", m.levelDbfs + _this->levelCalDb); tw::SameTextDisabled("(%.1f dBFS)", m.levelDbfs);
            } else {
                tw::TextColored(val, "%6.1f dBFS", m.levelDbfs);
            }
        } else { tw::TextDisabled("--"); }
        // fixed row (rows must not appear/disappear, the lines below would jump)
        tw::Text("RF status:"); tw::SameNext();
        if (!m.valid) { tw::TextDisabled("--"); }
        else if (m.peakDbfs > -3.0f) { tw::TextColored(bad, "ADC OVERLOAD (peak %.1f dBFS) - reduce gain", m.peakDbfs); }
        else if (m.levelDbfs > -15.0f) { tw::TextColored(warn, "level high (peak %.1f dBFS)", m.peakDbfs); }
        else if (m.levelDbfs < -45.0f) { tw::TextColored(warn, "level low (peak %.1f dBFS)", m.peakDbfs); }
        else { tw::TextColored(good, "OK (peak %.1f dBFS)", m.peakDbfs); }

        // MER / EVM
        tw::Text("MER:"); tw::SameNext();
        if (m.valid && m.symSync) {
            tw::TextColored(m.merDb >= 20.0f ? good : (m.merDb >= 12.0f ? warn : bad), "%5.1f dB", m.merDb); tw::SameText(" EVM rms"); tw::SameTextColored(val, "%4.1f %%", m.evmPct); tw::SameText(" peak"); tw::SameTextColored(val, "%4.0f %%", m.evmPeakPct);
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
            tw::TextColored(val, "%+7.1f Hz", m.carrierOffsetHz); tw::SameTextColored(std::fabs(m.carrierPpm) <= 0.2f ? good : (std::fabs(m.carrierPpm) <= 1.0f ? warn : bad), "(%+.3f ppm)", m.carrierPpm);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Reference: %s %.4f MHz\nIncludes the SDR's own frequency error.\nETSI: BS +-0.2 ppm, MS +-1 ppm (vs. received).",
                                  m.refFromSysinfo ? "SYSINFO DL" : "VFO", m.refFreqHz / 1e6);
            }
        } else { tw::TextDisabled("--"); }
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
            config.conf[_this->name]["levelCalDb"] = _this->levelCalDb;
            config.release(true);
            _this->meas.setLevelCal(_this->levelCalDb);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("dBm = dBFS + offset. Determine with a signal generator\nfor your SDR, gain setting and frequency. 0 = show dBFS.");
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
                        _this->carrierMon.startLog(lp.substr(0, lp.size() - 4) + "_carriers.csv", _this->levelCalDb);
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
        tw::Text("Signal constellation: ");
        if (tw::SameCheckbox(CONCAT("8 points (pi/4)##_tetra_c8_", _this->name), &_this->constellation8)) {
            config.acquire();
            config.conf[_this->name]["constellation8"] = _this->constellation8;
            config.release(true);
        }
        _this->drawOffsetCompCheckbox();
        {
            // one line if it fits, otherwise two columns under the label
            std::string l0 = std::string("Constellation##_tetra_cdet_") + _this->name;
            std::string l1 = std::string("Stations##_tetra_sdet_top_") + _this->name;
            std::string l2 = std::string("Network##_tetra_ndet_top_") + _this->name;
            std::string l3 = std::string("TX analysis##_tetra_tdet_top_") + _this->name;
            tw::CheckRow row;
            row.add(l0.c_str(), &_this->constDetached);
            row.add(l1.c_str(), &_this->subsDetached, _this->hamMode);   // stations: amateur radio mode only
            row.add(l2.c_str(), &_this->netDetached);
            row.add(l3.c_str(), &_this->txaDetached);
            std::string l4 = std::string("Trends##_tetra_trdet_top_") + _this->name;
            std::string l5 = std::string("Carriers##_tetra_crdet_top_") + _this->name;
            std::string l6 = std::string("Bursts##_tetra_bdet_top_") + _this->name;
            row.add(l4.c_str(), &_this->trendDetached);
            row.add(l5.c_str(), &_this->carrDetached);
            row.add(l6.c_str(), &_this->burstDetached);
            std::string l7 = std::string("Multipath##_tetra_mpdet_top_") + _this->name;
            row.add(l7.c_str(), &_this->mpDetached);
            int ch = row.draw("Own window:");
            if (ch >= 0) {
                config.acquire();
                config.conf[_this->name]["constDetached"] = _this->constDetached;
                config.conf[_this->name]["subsDetached"] = _this->subsDetached;
                config.conf[_this->name]["netDetached"] = _this->netDetached;
                config.conf[_this->name]["txaDetached"] = _this->txaDetached;
                config.conf[_this->name]["trendDetached"] = _this->trendDetached;
                config.conf[_this->name]["carrDetached"] = _this->carrDetached;
                config.conf[_this->name]["burstDetached"] = _this->burstDetached;
                config.release(true);
                if (ch == 4 && _this->trendDetached) { _this->focusTrendWin = true; }
                if (ch == 5 && _this->carrDetached) { _this->focusCarrWin = true; }
                if (ch == 6 && _this->burstDetached) { _this->focusBurstWin = true; }
                if (ch == 7) { _this->saveMpConfig(); if (_this->mpDetached) { _this->focusMpWin = true; } }
                if (ch == 1 && _this->subsDetached) { _this->focusSubsWin = true; }
                if (ch == 2 && _this->netDetached) { _this->focusNetWin = true; }
                if (ch == 3 && _this->txaDetached) { _this->focusTxaWin = true; }
            }
        }
        if (!_this->constDetached) {
            _this->drawConstellation(menuWidth);
        }

        if (ImGui::CollapsingHeader(CONCAT("Measurements##_tetra_meas_", _this->name), ImGuiTreeNodeFlags_DefaultOpen)) {
            if (_this->constDetached) {
                // shown below the constellation in its own window, not twice
                tw::TextDisabled("Shown in the constellation window.");
                if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_mfront_", _this->name))) { _this->focusConstWin = true; }
            } else {
                drawMeasurements(_this, menuWidth);
            }
        }
        if (ImGui::CollapsingHeader(CONCAT("TX analysis (spectrum)##_tetra_txa_", _this->name))) {
            if (_this->txaDetached) {
                tw::TextDisabled("Shown in its own window.");
                if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_tfront_", _this->name))) { _this->focusTxaWin = true; }
            } else {
                _this->drawTxAnalysis(menuWidth);
            }
        }
        if (ImGui::CollapsingHeader(CONCAT("Trends & timeslots##_tetra_trend_", _this->name))) {
            if (_this->trendDetached) {
                tw::TextDisabled("Shown in its own window.");
                if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_trfront_", _this->name))) { _this->focusTrendWin = true; }
            } else {
                _this->drawTrends(menuWidth);
            }
        }
        if (ImGui::CollapsingHeader(CONCAT("Carriers (multi-carrier)##_tetra_carr_", _this->name))) {
            if (_this->carrDetached) {
                tw::TextDisabled("Shown in its own window.");
                if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_crfront_", _this->name))) { _this->focusCarrWin = true; }
            } else {
                _this->drawCarriers(menuWidth);
            }
        }
        if (ImGui::CollapsingHeader(CONCAT("Burst analysis (uplink)##_tetra_burst_", _this->name))) {
            if (_this->burstDetached) {
                tw::TextDisabled("Shown in its own window.");
                if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_bfront_", _this->name))) { _this->focusBurstWin = true; }
            } else {
                _this->drawBursts(menuWidth);
            }
        }
        if (ImGui::CollapsingHeader(CONCAT("Multipath (downlink)##_tetra_mp_", _this->name))) {
            if (_this->mpDetached) {
                tw::TextDisabled("Shown in its own window.");
                if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_mpfront_", _this->name))) { _this->focusMpWin = true; }
            } else {
                _this->drawMultipath(menuWidth);
            }
        }
        if (ImGui::CollapsingHeader(CONCAT("Alarms##_tetra_alarm_", _this->name))) {
            _this->drawAlarms(menuWidth);
        }
        if (ImGui::CollapsingHeader(CONCAT("Report##_tetra_report_", _this->name))) {
            _this->drawReport(menuWidth);
        }
        if (ImGui::CollapsingHeader(CONCAT("GPS & coverage map##_tetra_gps_", _this->name))) {
            _this->drawGpsCoverage(menuWidth);
        }

        ImGui::BeginGroup();
        ImGui::Columns(2, CONCAT("TetraModeColumns##_", _this->name), false);
        if (ImGui::RadioButton(CONCAT("OSMO-TETRA##_", _this->name), _this->decoder_mode == 0) && _this->decoder_mode != 0) {
            _this->decoder_mode = 0; //osmo-tetra
            _this->setMode();
        }
        ImGui::NextColumn();
        if (ImGui::RadioButton(CONCAT("NETSYMS##_", _this->name), _this->decoder_mode == 1) && _this->decoder_mode != 1) {
            _this->decoder_mode = 1; //network symbol streaming
            _this->setMode();
        }
        ImGui::Columns(1, CONCAT("EndTetraModeColumns##_", _this->name), false);
        ImGui::EndGroup();

        if(_this->decoder_mode == 0) {
            //OSMO-TETRA: network & cell information (optionally in its own window)
            if (ImGui::CollapsingHeader(CONCAT("Network & cell##_tetra_net_", _this->name), ImGuiTreeNodeFlags_DefaultOpen)) {
                if (ImGui::Checkbox(CONCAT("Separate window##_tetra_ndet_", _this->name), &_this->netDetached)) {
                    config.acquire();
                    config.conf[_this->name]["netDetached"] = _this->netDetached;
                    config.release(true);
                    if (_this->netDetached) { _this->focusNetWin = true; }
                }
                if (_this->netDetached) {
                    tw::TextDisabled("Shown in its own window.");
                    if (tw::SameSmallButton(CONCAT("Bring to front##_tetra_nfront_", _this->name))) { _this->focusNetWin = true; }
                } else {
                    _this->drawNetworkBody(menuWidth);
                }
            }
            if (_this->hamMode) {
                _this->drawSubscribers(menuWidth);
            }
            _this->drawDuplexEditor(menuWidth);
        } else {
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
        if (!ImGui::CollapsingHeader(CONCAT("Duplex spacing table##_tetra_dup_", name))) { return; }

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
                              "Pull-in limited to +-6 kHz from where AFC started.\n"
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

        SourceGainInterface* g = findGainInterface();
        if (ImGui::Checkbox(CONCAT("Auto RF gain##_tetra_agc_", name), &autoGain)) {
            config.acquire();
            config.conf[name]["autoGain"] = autoGain;
            config.release(true);
            gainStatus.clear();
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Keeps the TETRA channel between -35 and -20 dBFS (peak below -6 dBFS)\n"
                              "by stepping the SDR gain slowly (one step every 4 s).\n"
                              "Supported: RTL-SDR (tuner AGC off), PlutoSDR (gain mode Manual),\n"
                              "Airspy (Sensitive or Linear mode), USRP, HackRF (LNA + VGA),\n"
                              "BladeRF (gain mode Manual), LimeSDR, SDRplay (IF gain, IF AGC off).\n"
                              "Spectran: set the reference level in the Spectran software.\n"
                              "Note: strong signals elsewhere in the SDR bandwidth can still overload\n"
                              "the ADC; watch the waterfall.");
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
    }

    // Called every frame from the GUI thread
    void controlLoops() {
        if (!enabled) { return; }
        auto now = std::chrono::steady_clock::now();
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
                // wanted gain change in dB: back to about -27 dBFS level / -10 dBFS peak,
                // at most 10 dB per step so the decoder keeps up
                double cur = g->gainStepDb(step);
                double want = cur;
                if (m.peakDbfs > -6.0f || m.levelDbfs > -20.0f) {
                    want = cur - std::clamp((std::max)((double)m.peakDbfs + 10.0, (double)m.levelDbfs + 27.0), 2.0, 10.0);
                }
                else if (m.levelDbfs < -35.0f && m.peakDbfs < -14.0f) {
                    want = cur + std::clamp((std::min)(-27.0 - (double)m.levelDbfs, -10.0 - (double)m.peakDbfs), 2.0, 10.0);
                }
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
                        std::string b = std::string(want > cur ? "RF gain up to " : "RF gain down to ") + g->gainLabel(best);
                        meas.addEvent(b);
                    }
                }
            }
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
        static const char* baudTxt = "4800\09600\019200\038400\057600\0115200\0";
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
            tw::SameTextColored(m.cellMatch == 1 ? good : bad, "%03d/%03d CC 0x%02x %.4f MHz", lock.mcc, lock.mnc, lock.cc, lock.dlFreq / 1e6);
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
    }

    // ---------------- Constellation & floating windows ----------------

    void drawConstellation(float width) {
        TetraMeasSnapshot m = meas.get();
        // with the offset removed from the display, judge the colour by the
        // offset-free EVM as well (EVM = 10^(-MER/20))
        float evm = compOriginOffset ? 100.0f * powf(10.0f, -m.merExclDb / 20.0f) : m.evmPct;
        bool good = m.valid && m.symSync && evm <= TetraConstellationView::RMS_LIMIT * 100.0f;
        // Colours follow the theme if it defines "Constellation*" keys (e.g. "RoSc 2")
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
                    row("Subscriber classes", "16-bit mask of subscriber classes allowed to access this cell.", "0x%04X", d->subscr_class);
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
            ImGui::SetTooltip("Colour code (decimal, 0..63) from the SYNC PDU = 0x%02X.\nIt is part of the scrambling code: if it were wrong,\nno other channel would pass its CRC.", _this->osmotetradecoder.getCc());
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
        double off = vfo->wtfVFO->centerOffset;
        if (!txa.isRunning()) {
            txa.setFftSize(4096 << txaFftIdx);
            txa.setAverages(txaAvg);
            txa.start(name + " TX analysis", off);
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
        if (ImGui::Checkbox(CONCAT("Run analysis (250 kHz tap)##_tetra_txarun_", name), &txaRun)) { saveTxaConfig(); }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Separate 250 kHz wide tap around the TETRA channel (not shown on the waterfall).\n"
                              "Costs some CPU; switch off when not needed.");
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
            if (ch) { saveTxaConfig(); }
            tw::TextDisabled("Measured through the TETRA filter (RRC, 18 ksym/s, a=0.35)\nin the adjacent channel and in the own channel.\nDefaults: typical ETSI TETRA values, please check against the standard.");
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
        b["timing"] = burstSt.timing;
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
        if (carrierMon.takeChanged()) { saveCarriers(); }
        double f = 0.0;
        if (burstSt.source == 0) { f = (double)meas.get().ulFreq; }
        else if (burstSt.source == 1) { f = burstSt.manualMHz * 1e6; }
        else { f = mainVfoFreq(); }
        double dlTap = (burstSt.timing && burstSt.source != 2) ? mainVfoFreq() : 0.0;
        burstAn.loop(enabled && burstRun, f, dlTap);
        chanAn.loop(enabled && mpRun && vfo != nullptr, mainVfoFreq());
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
        if (trendFile.render("##_tetra_trfile_" + name)) {
            std::string path = trendFile.expandString(trendFile.path);
            if (trendFile.pathIsValid()) {
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
            if (ImGui::BeginTabItem(CONCAT("Over time##_tetra_trt0_", name))) {
                trendView.draw("##_tetra_trplot_" + name, width, meas.trend(), constView.pal);
                if (!trendView.fileMode) {
                    tw::TextDisabled("Last 24 h kept in memory; longer: Start CSV log (Measurements)\nand open the file here later.");
                    drawTimeslotTable(width);
                }
                ImGui::EndTabItem();
            }
            bool evalTab = false;
            if (ImGui::BeginTabItem(CONCAT("Frequency stability##_tetra_trt1_", name))) {
                evalTab = true;
                updateEval();
                evalView.drawStability(name, width, constView.pal);
                ImGui::EndTabItem();
            }
            if (ImGui::BeginTabItem(CONCAT("Level vs. quality##_tetra_trt2_", name))) {
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
                        ok && m.symSync, levelCalDb, neighbourCarriers());
        if (carrierMon.takeChanged()) { saveCarriers(); }
    }

    void drawBursts(float width) {
        TetraMeasSnapshot m = meas.get();
        bool run = burstRun;
        if (burstAn.draw(name, width, constView.pal, burstSt, (double)m.ulFreq, mainVfoFreq(), levelCalDb, run)) {
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
        a["holdS"] = alarmSt.holdS; a["beep"] = alarmSt.beep; a["autoLog"] = alarmSt.autoLog;
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
        d.mainLevel = m.valid ? m.levelDbfs + levelCalDb : NAN;
        for (int i = 0; i < carrierMon.count(); i++) {
            const auto& c = carrierMon.at(i);
            if (!c.valid) { continue; }
            d.carriers.push_back({ c.label, c.freqHz / 1e6, c.level + levelCalDb, c.sync ? c.mer : NAN, c.sync });
        }
        auto subs = subscribers.snapshot();
        for (auto& sub : subs) { if (sub.kind == TetraSubscriber::KIND_GROUP) { d.groups++; } else { d.stations++; } }
        d.traffic = (int)subscribers.trafficSnapshot().size();
        for (auto& e : meas.getEvents()) { d.events.push_back({ e.t, e.text }); }
        d.crcLimit = evalView.crcLimit;
        std::string dir = logFolder.expandString(logFolder.path);
        if (!logFolder.pathIsValid() || dir.empty()) { return "!Log folder not valid (Measurements)"; }
        return TetraReport::write(dir, d);
    }

    void drawReport(float width) {
        tw::TextDisabled("One HTML file with the results of the session (summary,\ntrends, timeslots, TX and burst analysis, multipath,\ncarriers, events) in the log folder. Print it to PDF from\nthe browser.");
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
            if (!reportPath.empty() && reportPath[0] != '!') { TetraReport::openInBrowser(reportPath); }
        }
        if (!reportPath.empty()) {
            if (reportPath[0] == '!') { tw::TextColored(tc::bad(), "%s", reportPath.c_str() + 1); }
            else {
                tw::TextDisabled("%s", reportPath.c_str());
                if (tw::SameSmallButton(CONCAT("Open##_tetra_rep_open_", name))) { TetraReport::openInBrowser(reportPath); }
            }
        }
    }

    void floatingWindow(bool& detached, bool& focus, const char* title, const char* key, ImVec2 size, ImVec2 pos, void (TetraDemodulatorModule::*body)(float)) {
        if (!detached) { return; }
        ImGui::SetNextWindowSize(size, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowPos(pos, ImGuiCond_FirstUseEver);
        bool open = true;
        std::string t = std::string(title) + " (" + name + ")##_tetra_" + key + "_" + name;
        if (focus) { ImGui::SetNextWindowFocus(); focus = false; }
        if (ImGui::Begin(t.c_str(), &open)) {
            (this->*body)(ImGui::GetContentRegionAvail().x);
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
            bool open = true;
            std::string title = "TETRA TX analysis (" + name + ")##_tetra_twin_" + name;
            if (focusTxaWin) { ImGui::SetNextWindowFocus(); focusTxaWin = false; }
            if (ImGui::Begin(title.c_str(), &open)) {
                drawTxAnalysis(ImGui::GetContentRegionAvail().x);
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
            bool open = true;
            std::string title = "TETRA constellation (" + name + ")##_tetra_cwin_" + name;
            if (focusConstWin) { ImGui::SetNextWindowFocus(); focusConstWin = false; }
            if (ImGui::Begin(title.c_str(), &open)) {
                float w = ImGui::GetContentRegionAvail().x;
                ImGui::Checkbox(CONCAT("8 points (pi/4)##_tetra_c8w_", name), &constellation8);
                tw::SameLineFor(tw::checkboxWidth("Remove I/Q offset"));
                drawOffsetCompCheckbox("w");
                drawConstellation(w);
                drawMeasurements(this, w);
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
            bool open = true;
            std::string title = "TETRA network & cell (" + name + ")##_tetra_nwin_" + name;
            if (focusNetWin) { ImGui::SetNextWindowFocus(); focusNetWin = false; }
            if (ImGui::Begin(title.c_str(), &open)) {
                drawNetworkBody(ImGui::GetContentRegionAvail().x);
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
        if (!ImGui::CollapsingHeader(CONCAT("Stations & Traffic##_tetra_subs_", name), ImGuiTreeNodeFlags_DefaultOpen)) { return; }
        if (ImGui::Checkbox(CONCAT("Separate window##_tetra_sdet_", name), &subsDetached)) {
            config.acquire();
            config.conf[name]["subsDetached"] = subsDetached;
            config.release(true);
        }
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
        std::string stTitle = "Stations (" + std::to_string(stations.size()) + ")##_tetra_tab_st_" + name;
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
        std::string grTitle = "Groups (" + std::to_string(groups.size()) + ")##_tetra_tab_gr_" + name;
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
        std::string trTitle = "Traffic (" + std::to_string(traffic.size()) + ")##_tetra_tab_tr_" + name;
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
    std::string gainStatus;
    std::chrono::steady_clock::time_point gainLastStep;
    bool subsDetached = false;
    bool trafficHideControl = false;
    bool focusSubsWin = false;
    bool netDetached = false;
    bool focusNetWin = false;
    TetraTxAnalyzer txa;
    bool txaRun = false;
    bool txaDetached = false;
    bool focusTxaWin = false;
    int txaFftIdx = 1;        // 0: 4096, 1: 8192, 2: 16384
    int txaAvg = 20;
    float txaLimit[3] = { -60.0f, -70.0f, -70.0f };   // dBc at 25 / 50 / 75 kHz
    bool focusConstWin = false;
    // trends, further carriers, burst analysis (v0.18)
    TetraTrendView trendView;
    FileSelect trendFile { "%ROOT%/tetra_logs", { "TETRA CSV logs", "*.csv" } };
    std::string trendFileError;
    bool trendDetached = false, focusTrendWin = false;
    TetraCarrierMonitor carrierMon;
    bool carrDetached = false, focusCarrWin = false;
    TetraBurstAnalyzer burstAn;
    TetraBurstSettings burstSt;
    bool burstRun = false;
    bool burstDetached = false, focusBurstWin = false;
    std::atomic<double> pendingCarrierHz { 0.0 };
    TetraChannelAnalyzer chanAn;
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
    RadioIdDb radioid;
    float levelCalDb = 0.0f;
    FolderSelect logFolder;
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
