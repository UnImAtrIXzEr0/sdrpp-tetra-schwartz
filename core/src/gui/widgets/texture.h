#pragma once
// RGBA textures for modules that draw images into an ImDrawList (e.g. map
// tiles) without calling OpenGL themselves. Call only from the GUI thread.
#include <imgui.h>
#include <stdint.h>

namespace texture {
    // w x h pixels, 4 bytes (R, G, B, A) per pixel, rows without padding
    ImTextureID create(const uint8_t* rgba, int w, int h);
    void destroy(ImTextureID tex);
}
