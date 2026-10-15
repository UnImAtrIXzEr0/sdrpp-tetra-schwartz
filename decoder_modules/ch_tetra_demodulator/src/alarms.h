#pragma once
// Limit monitoring for long-term observation: level, MER, CRC errors, sync
// losses and the uplink noise. A condition has to last 'hold' seconds before
// it raises an alarm (and as long to clear it). Alarms and clears go to the
// event log; optionally a sound (Windows) and the CSV log is started.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <imgui.h>
#include <gui/style.h>
#include "tetra_colors.h"
#include "tetra_layout.h"

#ifdef _WIN32
#include <windows.h>
#pragma comment(lib, "user32.lib")
#endif

struct TetraAlarmSettings {
    bool enabled = false;
    bool levelOn = true;  float levelBelow = -90.0f;     // dBFS or dBm (calibrated)
    bool merOn = true;    float merBelow = 15.0f;
    bool crcOn = true;    float crcAbove = 5.0f;         // %
    bool syncOn = true;                                   // any sync loss / lock lost
    bool ulOn = false;    float ulAbove = -100.0f;       // uplink noise between bursts
    bool lossOn = true;                                   // samples lost (SDR overflow, timing slip)
    int holdS = 5;
    bool beep = true;
    bool autoLog = true;
};

class TetraAlarms {
public:
    enum Kind { A_LEVEL, A_MER, A_CRC, A_SYNC, A_UL, A_LOSS, A_COUNT };

    struct Input {
        bool valid = false;
        float level = NAN;      // calibrated if levelCal != 0
        float mer = NAN;
        float crcFail = NAN;    // % in the last second
        bool lockLost = false;  // no lock or sync loss in the last second
        float ulNoise = NAN;
        bool dataLoss = false;  // the SDR reported lost samples / timing slip in the last second
    };

    // once per second; returns texts for the event log
    std::vector<std::string> evaluate(const TetraAlarmSettings& st, const Input& in, bool& raised) {
        std::vector<std::string> ev;
        raised = false;
        if (!st.enabled || !in.valid) {
            for (int k = 0; k < A_COUNT; k++) { cnt[k] = 0; active[k] = false; }
            return ev;
        }
        bool cond[A_COUNT] = {
            st.levelOn && !std::isnan(in.level) && in.level < st.levelBelow,
            st.merOn && !std::isnan(in.mer) && in.mer < st.merBelow,
            st.crcOn && !std::isnan(in.crcFail) && in.crcFail > st.crcAbove,
            st.syncOn && in.lockLost,
            st.ulOn && !std::isnan(in.ulNoise) && in.ulNoise > st.ulAbove,
            st.lossOn && in.dataLoss
        };
        char b[160];
        for (int k = 0; k < A_COUNT; k++) {
            if (cond[k] != active[k]) { cnt[k]++; } else { cnt[k] = 0; }
            int need = ((k == A_SYNC || k == A_LOSS) && cond[k]) ? 1 : (std::max)(1, st.holdS);   // sync and sample losses count at once
            if (cnt[k] >= need) {
                active[k] = cond[k];
                cnt[k] = 0;
                if (active[k]) {
                    switch (k) {
                    case A_LEVEL: snprintf(b, sizeof(b), "ALARM level %.1f below %.1f", in.level, st.levelBelow); break;
                    case A_MER: snprintf(b, sizeof(b), "ALARM MER %.1f dB below %.1f dB", in.mer, st.merBelow); break;
                    case A_CRC: snprintf(b, sizeof(b), "ALARM CRC errors %.1f %% above %.1f %%", in.crcFail, st.crcAbove); break;
                    case A_SYNC: snprintf(b, sizeof(b), "ALARM lock / sync lost"); break;
                    case A_LOSS: snprintf(b, sizeof(b), "ALARM samples lost (SDR overflow / timing slip)"); break;
                    default: snprintf(b, sizeof(b), "ALARM uplink noise %.1f above %.1f", in.ulNoise, st.ulAbove); break;
                    }
                    raised = true;
                    count++;
                }
                else {
                    static const char* names[A_COUNT] = { "level", "MER", "CRC errors", "lock / sync", "uplink noise", "lost samples" };
                    snprintf(b, sizeof(b), "alarm cleared: %s", names[k]);
                }
                ev.push_back(b);
            }
        }
        if (raised && st.beep) {
#ifdef _WIN32
            MessageBeep(MB_ICONEXCLAMATION);
#endif
        }
        return ev;
    }

    bool any() const {
        for (int k = 0; k < A_COUNT; k++) { if (active[k]) { return true; } }
        return false;
    }
    std::string activeText() const {
        static const char* names[A_COUNT] = { "level", "MER", "CRC", "sync", "UL noise", "lost samples" };
        std::string s;
        for (int k = 0; k < A_COUNT; k++) {
            if (!active[k]) { continue; }
            if (!s.empty()) { s += ", "; }
            s += names[k];
        }
        return s;
    }
    int total() const { return count; }
    void resetCount() { count = 0; }

    // settings GUI; returns true when changed
    static bool draw(const std::string& id, float width, TetraAlarmSettings& st, bool calibrated) {
        bool ch = false;
        ch |= ImGui::Checkbox(("Monitor limits##_al_en_" + id).c_str(), &st.enabled);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Alarms and their end are written to the event log (and the CSV events\n"
                              "file while logging). A condition must last the hold time first.");
        }
        if (!st.enabled) { style::beginDisabled(); }
        float w = 80.0f * style::uiScale;
        auto row = [&](const char* label, bool* on, float* v, float step, const char* fmt) {
            ch |= ImGui::Checkbox((std::string(label) + "##_al_" + label + id).c_str(), on);
            if (v) {
                tw::SameLineFor(w);
                ImGui::SetNextItemWidth(w);
                ch |= ImGui::InputFloat((std::string("##_alv_") + label + id).c_str(), v, step, step * 5, fmt);
            }
        };
        row(calibrated ? "Level below (dBm)" : "Level below (dBFS)", &st.levelOn, &st.levelBelow, 1.0f, "%.0f");
        row("MER below (dB)", &st.merOn, &st.merBelow, 1.0f, "%.0f");
        row("CRC errors above (%)", &st.crcOn, &st.crcAbove, 0.5f, "%.1f");
        row("Lock / sync lost", &st.syncOn, nullptr, 0, nullptr);
        row(calibrated ? "UL noise above (dBm)" : "UL noise above (dBFS)", &st.ulOn, &st.ulAbove, 1.0f, "%.0f");
        if (st.ulOn) { tw::TextDisabled("UL noise needs the burst analysis on the uplink."); }
        row("Samples lost (overflow / slip)", &st.lossOn, nullptr, 0, nullptr);
        ImGui::SetNextItemWidth(w);
        if (ImGui::InputInt(("Hold time (s)##_al_hold_" + id).c_str(), &st.holdS, 1, 5)) { st.holdS = std::clamp(st.holdS, 1, 600); ch = true; }
        ch |= ImGui::Checkbox(("Start CSV log on alarm##_al_log_" + id).c_str(), &st.autoLog);
#ifdef _WIN32
        ch |= ImGui::Checkbox(("Sound##_al_beep_" + id).c_str(), &st.beep);
#endif
        if (!st.enabled) { style::endDisabled(); }
        return ch;
    }

private:
    bool active[A_COUNT] = {};
    int cnt[A_COUNT] = {};
    int count = 0;
};
