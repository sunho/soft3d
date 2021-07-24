#pragma once

#include <soft3d/common/threadpool.h>
#include <soft3d/renderer/interface.h>
#include <soft3d/scene/scene.h>

struct RTCPUConfig {
    RTCPUConfig() = default;
    int threadNum{ 12 };
    size_t maxWidth{ 1000 };
    size_t maxHeight{ 1000 };
    int maxRayHit{ 20 };
    int distSampleNum{ 4 };
    size_t pathSampleNum{ 10 };
    Float closeTime{ 0.0001f };
    Float closeEpsillon { 0.000001f };
};

struct RTCPURenderer : public Renderer {
    RTCPURenderer(RTCPUConfig conf);
    ~RTCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;
    int getProcessedRays() {
        return processedRays.load(std::memory_order_relaxed);
    }

  private:
    void renderPixel(const Vector2& pos, Image& screen);
    Vector3 rayColor(Ray ray, Float t0, Float t1, int depth = 0);
    Vector3 shadePhong(Ray ray, RayHit hit, const Material& shade, int depth);
    Vector3 shadeDielectric(Ray ray, RayHit hit, const Material& shade, int depth);
    bool refractRay(Ray ray, Vector3 normal, Float index, Vector3& out);
    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit);
    bool testSphereRay(const Vector3& e, Float radius, Ray ray, Float t0, Float t1, RayHit& hit);
    bool testTriangleRay(const Triangle3& triangle, Ray ray, Float t0, Float t1, RayHit& hit,
                         bool singleSide);
    std::vector<Vector2> generateJittered(int n);
    RTCPUConfig conf;
    ThreadPool<Vector2> threadPool;
    Scene scene;
    std::atomic<int> processedRays{ 0 };
};
