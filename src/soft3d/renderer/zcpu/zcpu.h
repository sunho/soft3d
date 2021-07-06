#pragma once

#include <soft3d/common/threadpool.h>
#include <soft3d/renderer/interface.h>
#include <soft3d/scene/scene.h>

struct ZCPUConfig {
    int threadNum;
    size_t maxWidth{ 1000 };
    size_t maxHeight{ 1000 };
    size_t shadowMapWidth{ 1000 };
    size_t shadowMapHeight{ 1000 };
    Float shadowNear{ 1.0f };
    Float shadowFar{ -1.0f };
};

using Shader = std::function<Vector3(const Vector3& bary, const Vector3& homo, const Geometry& geom,
                                     Float depth)>;

struct ZCPURenderer : public Renderer {
    ZCPURenderer();
    ~ZCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void renderInternal(Image& screen, const Matrix& mvp, const Shader& shader);

    void drawTriangle(Image& screen, const Triangle3& triangle, const Geometry& geom,
                      const Vector3& homo, const Shader& shader);
    void bakeShadowMap(Image& screen);

    Vector3 shadeSingleShadedTriangle(const Vector3& bary, const PlainTriangle& tri);
    Vector3 shadeTriangle(const Vector3& bary, const Triangle& tri, const Vector3& homo);
    void clearDepth();
    inline Float getDepth(int i, int j);
    void setDepth(int i, int j, Float depth);

    std::vector<Float> zBuffer;
    ThreadPool<std::tuple<int, int>> threadPool;
    std::map<int, TextureId> shadowMaps;
    std::map<int, Matrix> lightMvp;
    int width;
    Scene scene;
};
