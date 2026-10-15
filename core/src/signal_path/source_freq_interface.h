#pragma once
// Optional interface a source module can expose through
// ModuleManager::Instance::getInterface("SourceFreqCorrInterface") so that
// other modules (e.g. the TETRA demodulator's calibration) can read and set the
// reference oscillator correction. Convention as for the RTL-SDR "PPM
// correction": a positive value means the SDR's oscillator runs fast, i.e.
// without correction received signals appear too low in frequency.
// All calls happen from the GUI thread.

#include <string>

class SourceFreqCorrInterface {
public:
    virtual ~SourceFreqCorrInterface() = default;
    // Name of the source as registered with sigpath::sourceManager
    virtual std::string corrSourceName() = 0;
    // false while the device is not open
    virtual bool corrAvailable() = 0;
    virtual double corrPpm() = 0;
    virtual void setCorrPpm(double ppm) = 0;
    // smallest settable step in ppm (e.g. 1 for the RTL-SDR)
    virtual double corrResolutionPpm() { return 0.001; }
};
