#include <gui/widgets/texture.h>
#include <utils/opengl_include_code.h>

// OpenGL 1.2; the Windows headers only declare OpenGL 1.1
#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

namespace texture {
    ImTextureID create(const uint8_t* rgba, int w, int h) {
        GLuint id = 0;
        glGenTextures(1, &id);
        glBindTexture(GL_TEXTURE_2D, id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba);
        return (ImTextureID)(uintptr_t)id;
    }

    void destroy(ImTextureID tex) {
        GLuint id = (GLuint)(uintptr_t)tex;
        if (id) { glDeleteTextures(1, &id); }
    }
}
