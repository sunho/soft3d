#pragma once

#include <soft3d/image/image.h>
#include <soft3d/scene/scene.h>

struct Renderer {
    Renderer() = default;
    virtual ~Renderer() {
    }
    Renderer(const Renderer&) = default;
    Renderer& operator=(const Renderer&) = default;
    Renderer(Renderer&&) = default;
    Renderer& operator=(Renderer&&) = default;

    virtual Scene& sceneRef() = 0;
    virtual void render(Image& screen) = 0;
};
