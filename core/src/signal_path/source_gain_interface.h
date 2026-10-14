#pragma once
// Optional interface a source module can expose through
// ModuleManager::Instance::getInterface("SourceGainInterface") so that other
// modules (e.g. the TETRA demodulator's automatic gain) can step the RF gain.
// All calls happen from the GUI thread.

#include <string>

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
};
