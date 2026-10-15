#pragma once
// Optional interface a source module can expose through
// ModuleManager::Instance::getInterface("SourceGainInterface") so that other
// modules (e.g. the TETRA demodulator's automatic gain) can step the RF gain.
// All calls happen from the GUI thread.

#include <atomic>
#include <cstdio>
#include <string>
#include <gui/smgui.h>

class SourceGainInterface {
public:
    virtual ~SourceGainInterface() = default;
    // Name of the source as registered with sigpath::sourceManager
    virtual std::string gainSourceName() = 0;
    // false while the device is not open or a hardware AGC is active
    virtual bool gainControllable() = 0;
    virtual int gainStepCount() = 0;
    virtual int gainStep() = 0;
    virtual void setGainStep(int step) = 0;
    virtual float gainStepDb(int step) = 0;
    // Text for the gain at 'step' (default: "x.x dB"); sources whose steps are
    // not calibrated in dB can say so here
    virtual std::string gainLabel(int step) {
        char b[32];
        snprintf(b, sizeof(b), "%.1f dB", gainStepDb(step));
        return b;
    }
    // Lost-sample events of the device (ADC/USB overflows) since the start of
    // the stream; -1 if the source cannot tell
    virtual long long overflowCount() { return -1; }

    // Set by the module that steps the gain (TETRA auto RF gain) while it is
    // active: the source shows its gain controls locked. "Release" asks that
    // module to stop (gainReleaseReq, read and cleared by it).
    std::atomic<bool> gainLockedExt { false };
    std::atomic<bool> gainReleaseReq { false };

    // Around the gain widgets in the source menu
    void gainLockBegin() {
        if (gainLockedExt.load()) { SmGui::BeginDisabled(); }
    }
    void gainLockEnd(const std::string& id) {
        if (!gainLockedExt.load()) { return; }
        SmGui::EndDisabled();
        SmGui::TextColored(ImVec4(0.11f, 0.44f, 0.72f, 1.0f), "Locked by TETRA Auto RF gain");
        SmGui::SameLine();
        if (SmGui::Button(("Release##_gainlock_" + id).c_str())) { gainReleaseReq.store(true); }
    }
};
