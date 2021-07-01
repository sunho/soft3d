#pragma once

#include <focg/common/scene.h>
#include <focg/common/image.h>

struct Renderer {
    Renderer() = default;
    virtual ~Renderer() { }
    Renderer(const Renderer&) = default;
    Renderer& operator=(const Renderer&) = default;
    Renderer(Renderer&&) = default;
    Renderer& operator=(Renderer&&) = default;
    
    virtual Scene& sceneRef() = 0;
    virtual void render(Image& screen) = 0;
};
