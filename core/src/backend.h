#pragma once
#include <string>

namespace backend {
    int init(std::string resDir = "");
    void beginFrame();
    void render(bool vsync = true);
    void getMouseScreenPos(double& x, double& y);
    void setMouseScreenPos(double x, double y);
    // screen of the main window: resolution (pixels) and physical size (mm); false if unknown
    bool getScreenInfo(int& pxW, int& pxH, int& mmW, int& mmH);
    int renderLoop();
    int end();

}

