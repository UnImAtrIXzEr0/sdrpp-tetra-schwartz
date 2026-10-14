#include <core.h>
#include <iostream>
#include <stdio.h>
#include <module.h>
#include <ctm.h>
#include <utils/wav.h>
#include <utils/riff.h>
#include "dsp/types.h"
#include "dsp/multirate/polyphase_resampler.h"
#include "dsp/multirate/rational_resampler.h"

#include "ft8_etc/mshv_support.h"
#include "ft8_etc/mscore.h"
#include "ft8_etc/decoderms.h"
#include "symbolic.h"
#include <utils/wstr.h>
#include <utils/strings.h>

extern void doDecode(const char *mode, const char *path, int threads, std::function<void(int mode, std::vector<std::string> result)> callback);


static void help(const char *cmd) {
    fprintf(stderr,"usage: %s --decode <path> [--mode <mode>]\n", cmd);
    exit(1);
}

#ifdef _WIN32
int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR pCmdLine, int nCmdShow) {
    auto args = wstr::wstr2str(std::wstring(pCmdLine));
//    args = "--decode C:\\Temp\\sdrpp_ft8_mshv.wav.132 --mode ft8";
    std::vector<std::string> argsV;
    splitStringV(args, " ", argsV);
    std::vector<const char*> argv;
    argv.emplace_back("");
    for (auto& q : argsV) {
        argv.emplace_back(q.c_str());
    }
    auto argc = argv.size();
#else
int main(int argc, char* argv[]) {
#endif
    std::string decodeFile;
    std::string mode = "ft8";
    int threads = 1;
    for(int i=1; i<argc; i++) {
        if (!strcmp(argv[i],"--decode")) {
            i++;
            if (i < argc) {
                decodeFile = argv[i];
            }
        }
        if (!strcmp(argv[i],"--mode")) {
            i++;
            if (i < argc) {
                mode = argv[i];
            }
        }
        if (!strcmp(argv[i],"--threads")) {
            i++;
            if (i < argc) {
                threads = atoi(argv[i]);
                if (threads < 1 || threads > 8) {
                    threads = 1;
                }
            }
        }
    }
    if (false) {
        mode = "ft4";
        decodeFile = "C:\\Temp\\sdrpp_ft8_mshv.wav.236";
    }

    if (decodeFile == "") {
        fprintf(stderr, "ERROR: wav file for decode is not specified\n");
        help(argv[0]);
    }
    if (mode == "ft8" || mode == "ft4") {
        fprintf(stdout, "Using mode: %s\n", mode.c_str());
        fprintf(stdout, "Using file: %s\n", decodeFile.c_str());
        doDecode(mode.c_str(), decodeFile.c_str(), threads, [](int mode, std::vector<std::string> result) {
        });
        exit(0);
    } else {
        fprintf(stderr, "ERROR: invalid mode is specified. Valid modes: ft8, ft4\n");
        exit(1);
    }
}