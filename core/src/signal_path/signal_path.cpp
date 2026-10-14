#include <signal_path/signal_path.h>

namespace sigpath {
    Event<bool> txState;
    Event<float> averageTxSignalLevel;  // in range 0..1 where 1 matches max signal level on wire to SDR
    IQFrontEnd iqFrontEnd;
    VFOManager vfoManager;
    SourceManager sourceManager;
    SinkManager sinkManager;
    Transmitter *transmitter;

};