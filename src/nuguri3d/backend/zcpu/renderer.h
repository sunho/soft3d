#pragma once

#include <nuguri3d/backend/interface.h>
#include <nuguri3d/common/scene.h>
#include <nuguri3d/common/threadpool.h>

struct ZCPURenderer : public Renderer {
    ZCPURenderer();
    ~ZCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void drawPixel(Image& screen, const Vector2& pos, const Vector3& normal, const Shade& shade);
    void drawTriangle(Image& screen, const Triangle3& triangle, const Geometry& geom);
    Vector3 shadeSingleShadedTriangle(const Vector3& bary, const PlainTriangle& tri);
    Vector3 shadeTriangle(const Vector3& bary, const Triangle& tri);
    void clearDepth();
    inline Float getDepth(int i, int j);
    void setDepth(int i, int j, Float depth);
    std::vector<Float> zBuffer;
    ThreadPool<std::tuple<int, int>> threadPool;
    int width;
    Scene scene;
};
