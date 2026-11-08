#pragma once
// Optional interface of a source whose SDR can run either on an external
// frequency reference (10 MHz, GPSDO) or on its internal oscillator (USRP),
// exposed through ModuleManager::Instance::getInterface("SourceClockCalInterface").
// It calibrates the internal oscillator against the reference: the user asks
// for it in the source menu; a module that measures a received carrier precisely
// (the TETRA demodulator) performs it - carrier with the reference, carrier with
// the internal oscillator, difference = error of the internal oscillator - and
// hands the result back. A separate interface (not new methods of
// SourceFreqCorrInterface), so that sources built against the older header keep
// a matching vtable. All calls happen from the GUI thread.

#include <string>

class SourceClockCalInterface {
public:
    virtual ~SourceClockCalInterface() = default;
    // Name of the source as registered with sigpath::sourceManager
    virtual std::string clockCalSourceName() = 0;
    // true once after the user asked for a calibration (the request is consumed)
    virtual bool clockCalTakeRequest() = 0;
    // true once after the user cancelled a running calibration (consumed)
    virtual bool clockCalTakeCancel() = 0;
    // running on a reference (external / GPSDO): a calibration is possible
    virtual bool clockCalPossible() = 0;
    // switch the SDR to its internal oscillator (true, with the current
    // correction applied) or back to the selected reference (false); false = failed
    virtual bool clockCalUseInternal(bool internal) = 0;
    // correction applied with the internal oscillator, ppm (convention of
    // SourceFreqCorrInterface: positive = oscillator runs fast)
    virtual double clockCalPpm() = 0;
    // store the result for the device, with the time of the calibration
    virtual void clockCalStore(double ppm) = 0;
    // progress / result for the source menu; busy = a calibration is running
    virtual void clockCalStatus(const std::string& text, bool busy) = 0;
};
