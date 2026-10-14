#pragma once
#include "iq_frontend.h"
#include "vfo_manager.h"
#include "source.h"
#include "sink.h"
#include "trx.h"
#include <module.h>

namespace sigpath {
    SDRPP_EXPORT Event<bool> txState;
    SDRPP_EXPORT Event<float> averageTxSignalLevel;  // in range 0..1 where 1 matches max signal level on wire to SDR
    SDRPP_EXPORT IQFrontEnd iqFrontEnd;
    SDRPP_EXPORT VFOManager vfoManager;
    SDRPP_EXPORT SourceManager sourceManager;
    SDRPP_EXPORT SinkManager sinkManager;
    SDRPP_EXPORT Transmitter *transmitter;

};