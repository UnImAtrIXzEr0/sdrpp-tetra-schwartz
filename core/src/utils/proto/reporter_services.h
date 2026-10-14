#pragma once

#include <string>
#include <functional>
#include "http.h"
#include <utils/flog.h>

namespace net {

    enum ReportingSource {
        RS_RBN = 1,
        RS_WSPRNET_PULL = 2,
        RS_PSKREPORTER = 3,
        RS_PSKREPORTER_PULL = 4,
        RS_RBN_PULL = 5,
    };

    std::string to_string(net::ReportingSource source) {
        switch (source) {
            case net::RS_WSPRNET_PULL:
                return ("WSPR");
            case net::RS_PSKREPORTER:
                return ("FT8");
            case net::RS_PSKREPORTER_PULL:
                return ("FT8(p)");
            case net::RS_RBN:
                return ("CW");
            case net::RS_RBN_PULL:
                return ("CW(p)");
            default:
                return "???";
        }
    }



    struct Report {
        ReportingSource reportingSource;
        // either
        std::string errorStatus;
        //
        std::string reporterCallsign;
        std::string reportedCallsign;
        std::string timestamp;
        std::string mode;
        std::string modeParameters;
        std::string receiverLocator;
        int distance = -1;
        float decibel;
        float frequency; // in Hz
        long long createdTimestamp;

        std::string toString() const;
    };


    void getReportsFromRBN(const std::string telnetCallsign, const std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running);
    bool getReportsFromRBN_pull(const std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running);
    void getReportsFromPSKR(const std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running);
    bool getReportsFromPSKR_pull(const std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running);
    bool getReportsFromWSPR_pull(const std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running);

}