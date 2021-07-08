#pragma once

#include <soft3d/common/threadpool.h>
#include <soft3d/renderer/interface.h>
#include <soft3d/scene/scene.h>

struct ZCPUConfig {
    ZCPUConfig() = default;
    int threadNum{ 4 };
    size_t maxWidth{ 1000 };
    size_t maxHeight{ 1000 };
    size_t shadowMapWidth{ 1000 };
    size_t shadowMapHeight{ 1000 };
    Float shadowNear{ 1.0f };
    Float shadowFar{ -1.0f };
};

struct ZBuffer {
    ZBuffer() = default;
    void reset(int width, int height);
    inline Float get(int i, int j) const;
    inline void set(int i, int j, Float depth);

  private:
    std::vector<Float> data;
    int width;
    int height;
};

struct ShadowBuffer {
    ShadowBuffer() = delete;
    ShadowBuffer(Scene& scene, Float near, Float far);
    void addBakedShadow(int lightId, const Matrix& mvp, const TextureId& texId);
    inline bool shadowTest(int lightId, const Vector3& pos);
    void releaseAll();

  private:
    std::map<int, TextureId> shadowMaps;
    std::map<int, Matrix> lightMvp;
    Scene& scene;
    Float near;
    Float far;
};

struct FragmentInfo {
    int i;
    int j;
    Vector3 bary;
    Vector3 homo;
    Geometry geom;
    Float depth;
};

using Shader = std::function<Vector3(const Vector3& bary, const Vector3& homo, const Geometry& geom,
                                     Float depth)>;

struct ZCPURenderer : public Renderer {
    ZCPURenderer(ZCPUConfig conf);
    ~ZCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void renderPass(Image& screen, const Matrix& mvp, const Shader& shader);
    void addDrawTriangleJob(Image& screen, const Triangle3& triangle, const Geometry& geom,
                            const Vector3& homo, const Shader& shader);
    void populateShadowBuffer(Image& screen);

    Vector3 shadePlainTriangle(const Vector3& bary, const PlainTriangle& tri);
    Vector3 shadeTriangle(const Vector3& bary, const Triangle& tri, const Vector3& homo);

    ZCPUConfig conf;
    Scene scene;
    ZBuffer zBuffer;
    ShadowBuffer shadowBuffer;
    ThreadPool<FragmentInfo> threadPool;
    FragmentInfo* fragmentJobBuffer;
};
