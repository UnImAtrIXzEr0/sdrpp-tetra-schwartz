
#include "reporter_services.h"
#include "utils/flog.h"
#include "utils/strings.h"
#include "mqtt.h"
#include "json.hpp"
#include "utils/usleep.h"

namespace net {

    void generalReportError(const std::string &error, ReportingSource reportingSource, std::function<void(const ::net::Report &)> callback, bool &running, bool delay) {
        ::net::Report report;
        report.reportingSource = reportingSource;
        report.errorStatus = error;
        callback(report);
        if (delay) {
            for (int q = 0; q < 30; q++) {
                if (!running) {
                    break;
                }
                usleep(1000000);
            }
        }
    };


    // returns false after 3 minutes
    bool getReportsFromWSPR_pull(const std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running) {
        auto reportError = [&](const std::string &error, bool delay = true) {
            generalReportError(error, RS_WSPRNET_PULL, callback, running, delay);
        };
        auto startTime = currentTimeMillis();
        while(running) {
            net::http::Client httpClient;
            std::string formBuildId;
            try {
                reportError("Getting form..", false);
                auto emptyForm = httpClient.get("http://www.wsprnet.org:80/drupal/wsprnet/spotquery");
                reportError("Got form..", false);
                std::vector<std::string> lines;
                splitStringV(emptyForm, "\n", lines);
                // find string in lines, with form_id=wsprnet_spotquery_form
                for(int q=0; q<lines.size(); q++) {
                    if (lines[q].find("value=\"wsprnet_spotquery_form\"") != std::string::npos) {
                        // previous one contains the form_build_id
                        int index = lines[q - 1].find("value=\"form-");
                        if (index != std::string::npos) {
                            auto here = lines[q - 1].substr(index  + 7);
                            int index2 = here.find("\"");
                            if (index2 != std::string::npos) {
                                formBuildId = here.substr(0, index2);
                                break;
                            }

                        }
                    }

                }
                if (formBuildId == "") {
                    reportError("wsprnet http form not found");
                    continue;
                }
                auto formData = "band=All&mode=All&count=100&timelimit=3600&sortby=date&call=" + callsign + "&reporter=&sortrev=1&excludespecial=1&op=update&form_id=wsprnet_spotquery_form&form_build_id="+formBuildId;
                reportError("Sent query..", false);
                auto searchResults = httpClient.post("http://www.wsprnet.org:80/drupal/wsprnet/spotquery", formData);
                reportError("Got main response.", false);
                flog::info("post result:\n{}", searchResults);
                auto spots = searchResults.find("spots:</p>");
                if (spots == std::string::npos) {
                    reportError("unparseable response.");
                    continue;
                }
                searchResults = searchResults.substr(spots + 9);
                splitStringV(searchResults,"\n", lines);
                std::vector<std::string> fields;
                for(auto q=0; q<lines.size(); q++) {
                    auto line = lines[q];
                    if (line.find("<tr><td align='right'>&nbsp;20") == 0) {  // 20XX is year => this is a valid response.
                        removeSubstrings(line, "align='right'>");
                        removeSubstrings(line, "align='left'>");
                        removeSubstrings(line, "<tr>");
                        removeSubstrings(line, "</tr>");
                        removeSubstrings(line, "</td>");
                        removeSubstrings(line, "&nbsp;");
                        replaceSubstrings(line, "<td ","\t");
                        splitStringV(line, "\t", fields);
                        if (fields.size() >= 13) {
                            fields.erase(fields.begin());
                        }
                        flog::info("{}", line);
                        Report report;
                        report.timestamp = fields[0].substr(11)+":00Z";
                        report.reporterCallsign = fields[7];
                        report.reportedCallsign = fields[1];
                        report.frequency = atof(fields[2].c_str())*1000000;
                        report.reportingSource = RS_WSPRNET_PULL;
                        report.decibel = atof(fields[3].c_str());
                        report.mode = fields[11];
                        report.modeParameters = fields[4];  // drift
                        report.receiverLocator = fields[8]; // receiver loc
                        callback(report);
                    }
                }
                if (currentTimeMillis() - startTime > 300 * 1000) {
                    reportError("Stopped after 5 min", false);
                    running = false;
                    return false;
                    break;
                }
                for(int i=15; i>=0; i--) {
                    reportError("Done, sleeping " + std::to_string(i), false);
                    usleep(1000000);
                }
            } catch (std::exception &e) {
                reportError(e.what());
                continue;
            }
        }
        return true;

    }

    bool getReportsFromPSKR_pull(std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running) {
        auto reportError = [&](const std::string &error, bool delay = true) {
            generalReportError(error, RS_PSKREPORTER_PULL, callback, running, delay);
        };
        auto startTime = currentTimeMillis();
        while(running) {
            net::http::Client httpClient;
            std::string formBuildId;
            try {
                reportError("Requesting..", false);
                auto response = httpClient.get("https://pskreporter.info/cgi-bin/pskquery5.pl?encap=0&callback=doNothing&flowStartSeconds=-600&senderCallsign="+callsign);
                reportError("Got response..", false);
                int ix = response.find("{");
                if (ix == std::string::npos) {
                    reportError("Invalid response");
                    continue;
                }
                response = response.substr(ix);
                ix = response.rfind("}");
                if (ix == std::string::npos) {
                    reportError("Invalid response(2)");
                    continue;
                }
                response = response.substr(0, ix+1);
                auto obj = nlohmann::json::parse(response);
                int64_t ctm = currentTimeMillis()/1000;
                for(auto &[k, v] : obj["receptionReport"].items()) {
                    if ((int)v["isSender"] == 1) {
                        Report rep;
                        rep.reportingSource = ::net::RS_PSKREPORTER_PULL;
                        time_t t = (int) (float) v["flowStartSeconds"];
                        rep.reportedCallsign = v["senderCallsign"];
                        rep.reporterCallsign = v["receiverCallsign"];
                        rep.receiverLocator = v["receiverLocator"];
                        rep.mode = v["mode"];
                        rep.frequency = v["frequency"];
                        rep.decibel = v["sNR"];
                        if (ctm - t > 12 * 3600) {
                            rep.timestamp = "not recent";
                        } else {
                            auto tmm = std::gmtime(&t);
                            char streamTime[64];
                            strftime(streamTime, sizeof(streamTime), "%H:%M:%SZ", tmm);
                            rep.timestamp = streamTime;
                        }
                        callback(rep);
                    }
                }
                if (currentTimeMillis() - startTime > 300 * 1000) {
                    reportError("Stopped after 5 min", false);
                    running = false;
                    return false;
                    break;
                }
                for(int i=45; i>=0; i--) {
                    reportError("Done, sleeping " + std::to_string(i), false);
                    usleep(1000000);
                }
            } catch (std::exception &e) {
                reportError(e.what());
                continue;
            }
        }
        return true;

    }

    bool getReportsFromRBN_pull(std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running) {
        auto reportError = [&](const std::string &error, bool delay = true) {
            generalReportError(error, RS_RBN_PULL, callback, running, delay);
        };
        auto startTime = currentTimeMillis();
        while(running) {
            net::http::Client httpClient;
            std::string formBuildId;
            try {
                reportError("Querying endpoint..", false);
                auto jsonData = httpClient.get("https://reversebeacon.net/spots.php?h=fb94a8&b=91,3,7,84,12,17,22,27,32,37,42,50,55,62&s=0&r=40&cdx=" + callsign);
                reportError("Got data..", false);
                auto obj = nlohmann::json::parse(jsonData);
                int64_t ctm = currentTimeMillis()/1000;
                for(auto &[k, v] : obj["spots"].items()) {
                    Report rep;
                    time_t t = (int)(float)v[11];
                    rep.reportingSource = ::net::RS_RBN_PULL;
                    rep.reportedCallsign = v[2];
                    rep.reporterCallsign = v[0];
                    rep.mode = "CW";
                    rep.frequency = atof(((std::string)v[1]).c_str());
                    rep.decibel = v[3];
                    rep.modeParameters = std::to_string((int)v[4])+" WPM";
                    if (ctm - t > 12 * 3600) {
                        rep.timestamp = "not recent";
                    } else {
                        auto tmm = std::gmtime(&t);
                        char streamTime[64];
                        strftime(streamTime, sizeof(streamTime), "%H:%M:%SZ", tmm);
                        rep.timestamp = streamTime;
                    }
                    callback(rep);
                }
                if (currentTimeMillis() - startTime > 300 * 1000) {
                    reportError("Stopped after 5 min", false);
                    running = false;
                    return false;
                    break;
                }
                for(int i=15; i>=0; i--) {
                    reportError("Done, sleeping " + std::to_string(i), false);
                    usleep(1000000);
                }
            } catch (std::exception &e) {
                reportError(e.what());
                continue;
            }
        }
        return true;

    }

    void getReportsFromPSKR(std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running) {
        auto reportError = [&](std::string error) {
            ::net::Report report;
            report.reportingSource = ::net::RS_PSKREPORTER;
            report.errorStatus = error;
            callback(report);
            usleep(30000000);
        };
        Report rep;
        rep.reportingSource = RS_PSKREPORTER;
        rep.errorStatus = "Connecting...";
        callback(rep);
        while(running) {
            MQTTClient client;
            if (!client.socket_connect("mqtt.pskreporter.info", 1883)) {
                reportError("Cannot connect to pskreporter");
                continue;
            }
            client.receive_callback = [&](const char *topic, const uint8_t *payload, uint16_t len) {
                char buf[1024];
                memcpy(buf, payload, len);
                buf[len] = 0;
                try {
                    Report rep;
                    auto obj = nlohmann::json::parse(buf);
                    time_t t = obj["t"];
                    rep.reportingSource = ::net::RS_PSKREPORTER;
                    rep.reportedCallsign = obj["sc"];
                    rep.reporterCallsign = obj["rc"];
                    rep.decibel = obj["rp"];
                    rep.mode = obj["md"];
                    rep.receiverLocator = obj["rl"];
                    rep.frequency = obj["f"];
                    auto tmm = std::gmtime(&t);
                    char streamTime[64];
                    strftime(streamTime, sizeof(streamTime), "%H:%M:%SZ", tmm);
                    rep.timestamp = streamTime;
                    callback(rep);
                } catch (std::exception &e) {
                    return;
                }
            };
            if (!client.subscribe("pskr/filter/v2/+/FT8/" + callsign + "/+/+/+/+/+")) {
                client.stop();
                reportError("Could not send subscribe message to pskreporter, will retry");
                continue;
            }
            Report rep;
            rep.reportingSource = RS_PSKREPORTER;
            rep.errorStatus = "MQTT connected.";
            callback(rep);
            std::string lastStatus;
            client.onIdle = [&]() {
                if (client.gotPong) {
                    auto pongAgo = (currentTimeMillis() - client.gotPong) / 1000;
                    auto newStatus = "pong " + std::to_string(pongAgo)+" sec ago";
                    if (newStatus != lastStatus) {
                        lastStatus = newStatus;
                        ::net::Report report;
                        report.reportingSource = ::net::RS_PSKREPORTER;
                        report.errorStatus = newStatus;
                        callback(report);
                    }
                }
                if (!running && client.client->isOpen()) {
                    client.client->close();
                }
            };
            while (running) {
                if (!client.update()) {
                    break;
                }
            }
            reportError("Socket closed, will retry");
        }

    }

    void getReportsFromRBN(const std::string telnetCallsign, const std::string callsign, std::function<void(const ::net::Report &)> callback, bool &running) {
        auto reportError = [&](std::string error) {
            ::net::Report report;
            report.reportingSource = ::net::RS_RBN;
            report.errorStatus = error;
            callback(report);
            for(int q=0; q<30; q++) {
                if (!running) {
                    break;
                }
                usleep(1000000);
            }
        };
        while(running) {
            // download reports from reverse beacon network
            auto sock = net::connect("telnet.reversebeacon.net", 7000);
            uint8_t buf[10240];
            int received = sock->recv(buf, sizeof(buf), false, 10000);
            if (received < 0) {
                sock->close();
                reportError("receive: "+std::string(strerror(errno)));
                continue;
            }
            std::string data((char *) buf, received);
            if (data.find("your call") == std::string::npos) {
                sock->close();
                reportError("Protocol error");
                continue;
            }
            int snt = sock->send((const uint8_t *) (telnetCallsign + "\r\n").c_str(), telnetCallsign.size() + 2);
            if (snt < 0) {
                sock->close();
                reportError("Failed to send");
                continue;
            }
            std::string ready;
            int count = 0;
            generalReportError("telnet connected.", RS_RBN ,callback, running, false);
            while (running) {
                auto rd = sock->recv(buf, sizeof(buf), false, 1000);
                if (rd == 0) { // timeout?
                    if (!sock->isOpen()) {
                        break;
                    }
                    continue;
                }
                if (rd <= 0) {
                    flog::info("EOF from RBN");
                    sock->close();
                    return;
                }
                buf[rd] = 0;
                ready.append((const char *) buf);
                while (true) {
                    auto c = ready.find("\n");
                    if (c == std::string::npos) {
                        break;
                    }
                    auto line = ready.substr(0, c);
                    ready = ready.substr(c + 1);
                    if (line.size() != 0) {
                        if (line[line.length() - 1] == '\r') {
                            line.resize(line.length() - 1);
                        }
                    }
                    if (line.size() != 0) {
                        if (line.find("DX de") == 0) {
                            Report report;
                            std::vector<std::string> parts;
                            splitStringV(line, " ", parts);
                            coalesceSplit(parts);
                            if (parts.size() < 12) {
                                continue;
                            }
                            report.reportingSource = RS_RBN;
                            report.reporterCallsign = parts[2];
                            auto suff = report.reporterCallsign.find("-#");
                            if (suff != std::string::npos) {
                                report.reporterCallsign = report.reporterCallsign.substr(0, suff);
                            }
                            report.reportedCallsign = parts[4];
                            if (report.reportedCallsign == callsign || true) {
                                //                            report.reportedCallsign =
                            }
                            report.frequency = atof(parts[3].c_str());
                            report.mode = parts[5];
                            report.decibel = atof(parts[6].c_str());
                            report.modeParameters = parts[8] + " " + parts[9]; // 17 WPM
                            report.timestamp = parts[parts.size() - 1]; // 1530z
                            auto tspart = line.find(report.timestamp);
                            line = line.substr(0, tspart);
                            report.timestamp = report.timestamp.substr(0, 2) + ":" + report.timestamp.substr(2, 2) + ":00Z";
                            auto wpm = line.rfind("WPM");
                            if (wpm != std::string::npos) {
                                auto txt = line.substr(wpm + 4);
                                trimString(txt);
                                report.modeParameters = report.modeParameters + ": " + txt;
                            }
                            count++;
                            if (report.reportedCallsign.find(callsign) != std::string::npos || callsign == "") {
                                callback(report);
                            }
                            generalReportError("streaming: "+ std::to_string(count), RS_RBN ,callback, running, false);
                        }
                    }
                    // delete from ready
                }
            }
        }

    }


    std::string Report::toString() const {
        if (errorStatus != "") {
            return "STATUS:"+errorStatus;
        }
        return to_string(reportingSource)+": "+reportedCallsign+" by "+reporterCallsign+" TS:"+timestamp+" "+mode+" "+modeParameters+" LOC:"+receiverLocator+" FREQ:"+std::to_string(frequency)+" SNR:"+std::to_string(decibel);
    }
}

void test1() {
//    bool running = true;
//    net::getReportsFromRBN_pull("EA4HKF", [](const net::Report &rep) {
//        flog::info("RBN: {}", rep.toString());
//    }, running);
//    get
//    FT8ModuleInterface *ft8 = nullptr;
//    for(auto x: core::moduleManager.instances) {
//        ft8 = (FT8ModuleInterface *)x.second.instance->getInterface("FT8ModuleInterface");
//        if (ft8) {
//            break;
//        }
//    }
//    if (ft8) {
//        flog::info("FT8: {}", (void*)ft8);
//        auto [rv, msg] = ft8->encodeCQ_FT8("I9/UR6XXX","KO80", 1000);
//        FILE *f = fopen("/tmp/cq_ft8.raw", "wb");
//        fwrite(rv.data(), sizeof(dsp::stereo_t), rv.size(), f);
//        fclose(f);
//    }
}



