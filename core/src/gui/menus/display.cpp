
#include <gui/menus/display.h>
#include <imgui.h>
#include <gui/gui.h>
#include <core.h>
#include <gui/colormaps.h>
#include <gui/gui.h>
#include <gui/main_window.h>
#include <signal_path/signal_path.h>
#include <gui/style.h>
#include <utils/optionlist.h>
#include <algorithm>
#include <cmath>
#include <backend.h>

namespace displaymenu {
    bool showWaterfall;
    bool showFFT = true;
    bool showFFTShadows = true;
    bool showMicHistogram = false;
    bool fullWaterfallUpdate = true;
    bool showBattery = true;
    bool showClock = true;
    bool detectSignals = false;

    // Handler for center frequency changes
    EventHandler<double> centerFreqChangedHandler;
    std::string currentBatteryLevel = "?";
    int colorMapId = 0;
    std::vector<std::string> colorMapNames;
    std::string colorMapNamesTxt = "";
    std::string colorMapAuthor = "";
    int selectedWindow = 0;
    int fftRate = 20;
    int uiScaleId = 0;
    int transcieverLayoutId = 0;
    bool restartRequired = false;
    bool fftHold = false;
    int fftHoldSpeed = 60;
    bool fftSmoothing = false;
    int fftSmoothingSpeed = 100;
    // TETRA-SCHWARTZ: spectrum averaging as a time constant, max hold as a decay rate
    float fftAvgTau = 1.0f;      // s
    bool fftAvgPower = true;     // average the power (true) or the dB values
    float fftHoldDecay = 6.0f;   // dB/s, 0 = hold until switched off
    bool phoneLayout = false;
    bool touchMode() { return phoneLayout; }

    Event<ImGuiContext *> onDisplayDraw;

    TranscieverLayout transcieverLayout = TRAL_NONE;

#ifdef __ANDROID__
    float displayDensity = 1.0;  // 1.0 = 160 dpi. 3.5 = kinda high dpi etc. Coincides with good default scale
#endif
    bool snrSmoothing = false;
    int snrSmoothingSpeed = 20;

    OptionList<float, float> uiScales;

    const int FFTSizes[] = {
        524288,
        262144,
        131072,
        65536,
        32768,
        16384,
        8192,
        4096,
        2048,
        1024
    };

    const char* FFTSizesStr = "524288\0"
                              "262144\0"
                              "131072\0"
                              "65536\0"
                              "32768\0"
                              "16384\0"
                              "8192\0"
                              "4096\0"
                              "2048\0"
                              "1024\0"
#ifdef __APPLE__
                            "524288 ACCEL\0"
                            "262144 ACCEL\0"
                            "131072 ACCEL\0"
                            "65536 ACCEL\0"
                            "32768 ACCEL\0"
                            "16384 ACCEL\0"
                            "8192 ACCEL\0"
                            "4096 ACCEL\0"
                            "2048 ACCEL\0"
                            "1024 ACCEL\0"
#endif
        ;

    int fftSizeId = 0;

    const IQFrontEnd::FFTWindow fftWindowList[] = {
        IQFrontEnd::FFTWindow::RECTANGULAR,
        IQFrontEnd::FFTWindow::BLACKMAN,
        IQFrontEnd::FFTWindow::NUTTALL,
    };

    void updateFFTSpeeds() {
        // per spectrum: decay in dB, averaging weight from the time constant
        gui::waterfall.setFFTHoldSpeed(fftHoldDecay / (float)fftRate);
        gui::waterfall.setFFTSmoothingSpeed((float)(1.0 - std::exp(-1.0 / ((double)fftRate * (double)fftAvgTau))));
        gui::waterfall.setFFTSmoothingPower(fftAvgPower);
        gui::waterfall.setSNRSmoothingSpeed(std::min<float>((float)snrSmoothingSpeed / (float)(fftRate * 10.0f), 1.0f));
    }

    void init() {
        if (core::configManager.conf.contains("showFFT")) {
            showFFT = core::configManager.conf["showFFT"];
        }
        if (core::configManager.conf.contains("showFFTShadows")) {
            showFFTShadows = core::configManager.conf["showFFTShadows"];
        }
        if (core::configManager.conf.contains("showMicHistogram")) {
            showMicHistogram = core::configManager.conf["showMicHistogram"];
        }
        showWaterfall = core::configManager.conf["showWaterfall"];
        showWaterfall ? gui::waterfall.showWaterfall() : gui::waterfall.hideWaterfall();
        std::string colormapName = core::configManager.conf["colorMap"];
        if (colormaps::maps.find(colormapName) != colormaps::maps.end()) {
            colormaps::Map map = colormaps::maps[colormapName];
            gui::waterfall.updatePalletteFromArray(map.map, map.entryCount);
        }

        for (auto const& [name, map] : colormaps::maps) {
            colorMapNames.push_back(name);
            colorMapNamesTxt += name;
            colorMapNamesTxt += '\0';
            if (name == colormapName) {
                colorMapId = (colorMapNames.size() - 1);
                colorMapAuthor = map.author;
            }
        }

        fullWaterfallUpdate = core::configManager.conf["fullWaterfallUpdate"];
        gui::waterfall.setFullWaterfallUpdate(fullWaterfallUpdate);

        if (core::configManager.conf.contains("showBattery")) {
            showBattery = core::configManager.conf["showBattery"];
        }
        if (core::configManager.conf.contains("showClock")) {
            showClock = core::configManager.conf["showClock"];
        }
        if (core::configManager.conf.contains("detectSignals")) {
            detectSignals = core::configManager.conf["detectSignals"];
            sigpath::iqFrontEnd.togglePreprocessor(&sigpath::iqFrontEnd.detectorPreprocessor, detectSignals);
        }

        // Connect to center frequency change event
        sigpath::sourceManager.onTuneChanged.bindHandler(&centerFreqChangedHandler);
        centerFreqChangedHandler.ctx = NULL;
        centerFreqChangedHandler.handler = [](double freq, void* ctx) {
            sigpath::iqFrontEnd.detectorPreprocessor.setCenterFrequency(freq);
        };

        fftSizeId = 4;
        int fftSize = core::configManager.conf["fftSize"];
        for (int i = 0; i < std::size(FFTSizes); i++) {
            if (fftSize == FFTSizes[i]) {
                fftSizeId = i;
                break;
            }
        }
        auto accel = core::configManager.conf.contains("fftAccel") && core::configManager.conf["fftAccel"];
        if (accel) {
            fftSizeId += std::size(FFTSizes);
        }

        enableAcceleratedFFT = accel;
        sigpath::iqFrontEnd.setFFTSize(FFTSizes[fftSizeId % std::size(FFTSizes)]);

        fftRate = core::configManager.conf["fftRate"];
        sigpath::iqFrontEnd.setFFTRate(fftRate);

        selectedWindow = std::clamp<int>((int)core::configManager.conf["fftWindow"], 0, (sizeof(fftWindowList) / sizeof(IQFrontEnd::FFTWindow)) - 1);
        sigpath::iqFrontEnd.setFFTWindow(fftWindowList[selectedWindow]);

        gui::menu.locked = core::configManager.conf["lockMenuOrder"];

        phoneLayout = core::configManager.conf["smallScreen"];
        transcieverLayout = core::configManager.conf["transcieverLayout"];

        fftHold = core::configManager.conf["fftHold"];
        fftHoldSpeed = core::configManager.conf["fftHoldSpeed"];
        gui::waterfall.setFFTHold(fftHold);
        fftSmoothing = core::configManager.conf["fftSmoothing"];
        fftSmoothingSpeed = core::configManager.conf["fftSmoothingSpeed"];
        // new settings; older configs: convert the speed numbers (alpha = speed / (rate * 10) per spectrum,
        // i.e. a time constant of 10 / speed seconds; hold: speed / 10 dB/s)
        if (core::configManager.conf.contains("fftAvgTau")) { fftAvgTau = core::configManager.conf["fftAvgTau"]; }
        else { fftAvgTau = 10.0f / (float)std::max<int>(fftSmoothingSpeed, 1); }
        if (core::configManager.conf.contains("fftAvgPower")) { fftAvgPower = core::configManager.conf["fftAvgPower"]; }
        if (core::configManager.conf.contains("fftHoldDecay")) { fftHoldDecay = core::configManager.conf["fftHoldDecay"]; }
        else { fftHoldDecay = (float)fftHoldSpeed / 10.0f; }
        fftAvgTau = std::clamp<float>(fftAvgTau, 0.05f, 60.0f);
        fftHoldDecay = std::clamp<float>(fftHoldDecay, 0.0f, 200.0f);
        gui::waterfall.setFFTSmoothing(fftSmoothing);
        snrSmoothing = core::configManager.conf["snrSmoothing"];
        snrSmoothingSpeed = core::configManager.conf["snrSmoothingSpeed"];
        gui::waterfall.setSNRSmoothing(snrSmoothing);
        updateFFTSpeeds();


        // Define and load UI scales

        std::vector<float> scales = {0.25f, 0.5f, 0.66f, 0.75f, 0.9f, 1.0f, 1.25f, 1.5f, 1.75f, 2.0f, 2.5f, 3.0f, 4.0f};
        bool hasNativeScale = false;
#ifdef __ANDROID__
        for (int i = 0; i < scales.size(); i++) {
            float scale = scales[i];
            if (scale == displayDensity) {
                hasNativeScale = true;
            }
        }
        if (!hasNativeScale) {
            uiScales.define(displayDensity, std::to_string((int)(displayDensity * 100)) + "% (native)", displayDensity);
        }
#endif
        for (int i = 0; i < scales.size(); i++) {
            float scale = scales[i];
            uiScales.define(scale, std::to_string((int)(scale * 100)) + "%", scale);
        }

        uiScaleId = uiScales.valueId(style::uiScale);
    }

    void setWaterfallShown(bool shown) {
        showWaterfall = shown;
        showWaterfall ? gui::waterfall.showWaterfall() : gui::waterfall.hideWaterfall();
        core::configManager.acquire();
        core::configManager.conf["showWaterfall"] = showWaterfall;
        core::configManager.release(true);
    }


    void checkKeybinds() {
        if (ImGui::IsKeyPressed(ImGuiKey_Home, false)) {
            setWaterfallShown(!showWaterfall);
        }
    }


    void draw(void* ctx) {
        float menuWidth = ImGui::GetContentRegionAvail().x;
        if (ImGui::Checkbox("Touch mode (larger controls)##_sdrpp", &phoneLayout)) {
            core::configManager.acquire();
            core::configManager.conf["smallScreen"] = phoneLayout;
            core::configManager.release(true);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("For touch screens and small screens: larger buttons, sliders and scroll bars,\n"
                              "extra touch margin, drag with the finger to scroll the menu.\n"
                              "A tap shows the help text of a value (instead of hovering).");
        }
        {
            // scale suggestion from the physical screen size (10-17 inch touch laptops)
            int pw, ph, mw, mh;
            if (backend::getScreenInfo(pw, ph, mw, mh)) {
                double inch = std::sqrt((double)mw * mw + (double)mh * mh) / 25.4;
                double dpi = pw / (mw / 25.4);
                // keep controls about the same physical size: 100 % at ~110 dpi
                float sug = (float)std::clamp(std::round(dpi / 110.0 * 4.0) / 4.0, 1.0, 3.0);
                ImGui::TextDisabled("Screen %.1f\", %dx%d, %.0f dpi: suggested %d %%", inch, pw, ph, dpi, (int)std::lround(sug * 100.0f));
                if (std::fabs(sug - style::uiScale) > 0.01f) {
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Use##_sdrpp_sugscale")) {
                        core::configManager.acquire();
                        core::configManager.conf["uiScale"] = sug;
                        core::configManager.release(true);
                        if (uiScales.keyExists(sug)) { uiScaleId = uiScales.keyId(sug); }
                        restartRequired = true;
                    }
                }
            }
        }
        if (ImGui::RadioButton("Layout: default ##_sdrpp", transcieverLayout == TRAL_NONE)) {
            core::configManager.acquire();
            transcieverLayout = TRAL_NONE;
            core::configManager.conf["transcieverLayout"] = transcieverLayout;
            core::configManager.release(true);
        }
        ImGui::SameLine();
        if (ImGui::RadioButton("SSB trx ##_sdrpp", transcieverLayout == TRAL_SSB_FIRST)) {
            core::configManager.acquire();
            transcieverLayout = TRAL_SSB_FIRST;
            core::configManager.conf["transcieverLayout"] = transcieverLayout;
            core::configManager.release(true);
        }

        if (ImGui::Checkbox("Waterfall##_sdrpp", &showWaterfall)) {
            setWaterfallShown(showWaterfall);
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("FFT##_sdrpp", &showFFT)) {
            core::configManager.acquire();
            core::configManager.conf["showFFT"] = showFFT;
            core::configManager.release(true);
        }
        ImGui::SameLine();
        if (ImGui::Checkbox("Shadow##_sdrpp", &showFFTShadows)) {
            core::configManager.acquire();
            core::configManager.conf["showFFTShadows"] = showFFTShadows;
            core::configManager.release(true);
        }

        if (ImGui::Checkbox("Full Waterfall Update##_sdrpp", &fullWaterfallUpdate)) {
            gui::waterfall.setFullWaterfallUpdate(fullWaterfallUpdate);
            core::configManager.acquire();
            core::configManager.conf["fullWaterfallUpdate"] = fullWaterfallUpdate;
            core::configManager.release(true);
        }
#ifdef __ANDROID__
        if (ImGui::Checkbox("Show Battery##_sdrpp", &showBattery)) {
            gui::waterfall.setFullWaterfallUpdate(fullWaterfallUpdate);
            core::configManager.acquire();
            core::configManager.conf["showBattery"] = showBattery;
            core::configManager.release(true);
        }
#endif
        if (ImGui::Checkbox("Show Clock##_sdrpp", &showClock)) {
            gui::waterfall.setFullWaterfallUpdate(fullWaterfallUpdate);
            core::configManager.acquire();
            core::configManager.conf["showClock"] = showClock;
            core::configManager.release(true);
        }
#if 0
        if (ImGui::Checkbox("Detect Signals##_sdrpp", &detectSignals)) {
            sigpath::iqFrontEnd.togglePreprocessor(&sigpath::iqFrontEnd.detectorPreprocessor, detectSignals);
            core::configManager.acquire();
            core::configManager.conf["detectSignals"] = detectSignals;
            core::configManager.release(true);
        }
#endif

        if (ImGui::Checkbox("Lock Menu Order##_sdrpp", &gui::menu.locked)) {
            core::configManager.acquire();
            core::configManager.conf["lockMenuOrder"] = gui::menu.locked;
            core::configManager.release(true);
        }
        // TETRA-SCHWARTZ: spectrum averaging (time constant) and max hold (decay rate)
        if (ImGui::Checkbox("Spectrum averaging##_sdrpp", &fftSmoothing)) {
            gui::waterfall.setFFTSmoothing(fftSmoothing);
            core::configManager.acquire();
            core::configManager.conf["fftSmoothing"] = fftSmoothing;
            core::configManager.release(true);
        }
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Exponential average of the spectrum above the waterfall (the waterfall itself is not averaged).\n"
                              "Time constant: after this time a step has reached 63 %%, after 5x the time 99 %%.\n"
                              "Power: averages the power - correct levels, use this for comparisons.\n"
                              "Log: averages the dB values (as before) - noise reads about 2.5 dB too low.\n"
                              "For measurements use TX analysis (spectrum), it averages a fixed number of spectra.");
        }
        ImGui::SameLine();
        ImGui::FillWidth();
        if (ImGui::InputFloat("##sdrpp_fft_avg_tau", &fftAvgTau, 0.1f, 1.0f, "%.2f s")) {
            fftAvgTau = std::clamp<float>(fftAvgTau, 0.05f, 60.0f);
            updateFFTSpeeds();
            core::configManager.acquire();
            core::configManager.conf["fftAvgTau"] = fftAvgTau;
            core::configManager.release(true);
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Time constant of the spectrum averaging (0.05 ... 60 s)"); }
        ImGui::Indent();
        {
            bool newPower = fftAvgPower;
            if (ImGui::RadioButton("Power##_sdrpp_avg_pow", fftAvgPower)) { newPower = true; }
            ImGui::SameLine();
            if (ImGui::RadioButton("Log (dB)##_sdrpp_avg_log", !fftAvgPower)) { newPower = false; }
            if (newPower != fftAvgPower) {
                fftAvgPower = newPower;
                updateFFTSpeeds();
                core::configManager.acquire();
                core::configManager.conf["fftAvgPower"] = fftAvgPower;
                core::configManager.release(true);
            }
        }
        ImGui::Unindent();

        if (ImGui::Checkbox("Max hold##_sdrpp", &fftHold)) {
            gui::waterfall.setFFTHold(fftHold);
            core::configManager.acquire();
            core::configManager.conf["fftHold"] = fftHold;
            core::configManager.release(true);
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Second trace with the maximum of the spectrum; it falls back with the decay rate.\nSwitch off and on to clear it."); }
        ImGui::SameLine();
        ImGui::FillWidth();
        if (ImGui::InputFloat("##sdrpp_fft_hold_decay", &fftHoldDecay, 1.0f, 10.0f, "%.1f dB/s")) {
            fftHoldDecay = std::clamp<float>(fftHoldDecay, 0.0f, 200.0f);
            updateFFTSpeeds();
            core::configManager.acquire();
            core::configManager.conf["fftHoldDecay"] = fftHoldDecay;
            core::configManager.release(true);
        }
        if (ImGui::IsItemHovered()) { ImGui::SetTooltip("Decay of the max hold trace in dB per second (0 = hold until switched off)"); }

        if (ImGui::Checkbox("SNR Smoothing##_sdrpp", &snrSmoothing)) {
            gui::waterfall.setSNRSmoothing(snrSmoothing);
            core::configManager.acquire();
            core::configManager.conf["snrSmoothing"] = snrSmoothing;
            core::configManager.release(true);
        }
        ImGui::SameLine();
        ImGui::FillWidth();
        if (ImGui::InputInt("##sdrpp_snr_smoothing_speed", &snrSmoothingSpeed)) {
            snrSmoothingSpeed = std::max<int>(snrSmoothingSpeed, 1);
            updateFFTSpeeds();
            core::configManager.acquire();
            core::configManager.conf["snrSmoothingSpeed"] = snrSmoothingSpeed;
            core::configManager.release(true);
        }

        ImGui::LeftLabel("High-DPI Scaling");
        ImGui::FillWidth();
        if (ImGui::Combo("##sdrpp_ui_scale", &uiScaleId, uiScales.txt)) {
            core::configManager.acquire();
            core::configManager.conf["uiScale"] = uiScales[uiScaleId];
            core::configManager.release(true);
            restartRequired = true;
        }

        ImGui::LeftLabel("FFT Framerate");
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::InputInt("##sdrpp_fft_rate", &fftRate, 1, 10)) {
            fftRate = std::max<int>(1, fftRate);
            sigpath::iqFrontEnd.setFFTRate(fftRate);
            updateFFTSpeeds();
            core::configManager.acquire();
            core::configManager.conf["fftRate"] = fftRate;
            core::configManager.release(true);
        }

        ImGui::LeftLabel("FFT Size");
        auto textSize8888  =  ImGui::CalcTextSize("88888888");
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX() - textSize8888.x);
        if (ImGui::Combo("##sdrpp_fft_size", &fftSizeId, FFTSizesStr)) {
            enableAcceleratedFFT = fftSizeId >= std::size(FFTSizes);
            sigpath::iqFrontEnd.setFFTSize(FFTSizes[fftSizeId % std::size(FFTSizes)]);
            core::configManager.acquire();
            core::configManager.conf["fftSize"] = FFTSizes[fftSizeId % std::size(FFTSizes)];
            core::configManager.conf["fftAccel"] = fftSizeId >= std::size(FFTSizes);
            core::configManager.release(true);
        }

        {
            ImGui::SameLine();
            static auto lastFFTReportTime = currentTimeMillis();
            static long long lastFFTReport = 0;
            auto ctm = currentTimeMillis();
            if (ctm - lastFFTReportTime > 1000) {
                lastFFTReportTime += 1000;
                lastFFTReport = fftCumulativeTime;
                fftCumulativeTime = 0;
            }
            ImGui::Text("%lld ms/s", lastFFTReport);
        }

        ImGui::LeftLabel("FFT Window");
        ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
        if (ImGui::Combo("##sdrpp_fft_window", &selectedWindow, "Rectangular\0Blackman\0Nuttall\0")) {
            sigpath::iqFrontEnd.setFFTWindow(fftWindowList[selectedWindow]);
            core::configManager.acquire();
            core::configManager.conf["fftWindow"] = selectedWindow;
            core::configManager.release(true);
        }

        if (colorMapNames.size() > 0) {
            ImGui::LeftLabel("Color Map");
            ImGui::SetNextItemWidth(menuWidth - ImGui::GetCursorPosX());
            if (ImGui::Combo("##_sdrpp_color_map_sel", &colorMapId, colorMapNamesTxt.c_str())) {
                colormaps::Map map = colormaps::maps[colorMapNames[colorMapId]];
                gui::waterfall.updatePalletteFromArray(map.map, map.entryCount);
                core::configManager.acquire();
                core::configManager.conf["colorMap"] = colorMapNames[colorMapId];
                core::configManager.release(true);
                colorMapAuthor = map.author;
            }
            ImGui::Text("Color map Author: %s", colorMapAuthor.c_str());
        }

        onDisplayDraw.emit(GImGui);

        if (restartRequired) {
            ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Restart required.");
        }
    }
}
