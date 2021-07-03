#pragma once

#include <nuguri3d/backend/interface.h>
#include <nuguri3d/common/scene.h>

struct ZCPURenderer : public Renderer {
    ZCPURenderer();
    ~ZCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void drawPixel(Image& screen, const Vector2& pos, const Vector3& normal, const Shade& shade);
    void drawTriangle(Image& screen, const Triangle& triangle, const Triangle& original);
    void clearDepth();
    inline Float getDepth(int i, int j);
    void setDepth(int i, int j, Float depth);
    std::vector<Float> zBuffer;
    int width;
    Scene scene;
};
