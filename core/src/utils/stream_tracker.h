#pragma once

#include <utils/flog.h>
#include <string>
#include <ctm.h>


struct StreamTracker {
    int lastCount = 0;
    int lastCountSecond = 0;
    std::string title;

    std::vector<int> queue;
    float xavg = 0;

    StreamTracker(const std::string &title) : title(title) {
    }

    void add(int count) {
        auto ctm = currentTimeMillis();
        if (lastCountSecond != ctm / 1000) {
            lastCountSecond = ctm / 1000;
            queue.emplace_back(lastCount);
            while(queue.size() > 100) {
                queue.erase(queue.begin());
            }
            int per8second = 0;
            int per30second = 0;
            {
                int sum = 0;
                int cnt = 0;
                for (int q = 0; q < 8; q++) {
                    if (q < queue.size()) {
                        sum += queue[queue.size() - q - 1];
                        cnt++;
                    }
                }
                per8second = sum/cnt;
            }
            {
                int sum = 0;
                int cnt = 0;
                for (int q = 0; q < 30; q++) {
                    if (q < queue.size()) {
                        sum += queue[queue.size() - q - 1];
                        cnt++;
                    }
                }
                per30second = sum/cnt;
            }
            if (queue.size() == 8) {
                xavg = per8second;
            }
            xavg = xavg * 0.9f + lastCount * 0.1f;
            flog::info("Stream tracker: {}  per second: {}  xavg: {} per 8 sec: {}  per 30 sec: {}", title, lastCount, xavg, per8second, per30second);
            lastCount = 0;
        }
        lastCount += count;
    }


};