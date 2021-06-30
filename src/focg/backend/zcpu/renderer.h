#pragma once

#include <focg/common/scene.h>
#include <focg/backend/interface.h>

struct ZCPURenderer : public Renderer {
    ZCPURenderer();
    ~ZCPURenderer();
    
    Scene& sceneRef() override;
    void render(Screen& screen) override;

private:
    void drawTriangle(Screen& screen, const Triangle& triangle, const Triangle &original);
    void clearDepth();
    Float getDepth(int i, int j);
    void setDepth(int i, int j, Float depth);
    std::vector<Float> zBuffer;
    int width;
    Scene scene;
};
