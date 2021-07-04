#pragma once

#include <soft3d/backend/interface.h>
#include <soft3d/common/scene.h>
#include <soft3d/common/threadpool.h>

struct Shader {
    std::function<Vector3(const Vector3& bary, const Vector3& homo, const Geometry& geom,
                          Float depth)>
        func;
};

struct ZCPURenderer : public Renderer {
    ZCPURenderer();
    ~ZCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void renderInternal(Image& screen, const Matrix& mvp, Shader& shader);

    void drawTriangle(Image& screen, const Triangle3& triangle, const Geometry& geom,
                      const Vector3& homo, Shader& shader);
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
