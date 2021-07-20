#pragma once

#include <soft3d/common/threadpool.h>
#include <soft3d/renderer/interface.h>
#include <soft3d/scene/scene.h>

struct RTCPUConfig {
    RTCPUConfig() = default;
    int threadNum{ 10 };
    size_t maxWidth{ 1000 };
    size_t maxHeight{ 1000 };
    int maxRayHit{ 20 };
    int distSampleNum{ 4 };
    bool antialias{ true };
    bool usePathtracing{ true };
    Float closeTime{ 0.0001f };
};

struct RTCPURenderer : public Renderer {
    RTCPURenderer(RTCPUConfig conf);
    ~RTCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void renderPixel(const Vector2& pos, Image& screen);
    Vector3 rayColor(Ray ray, Float t0, Float t1, const std::vector<Vector2>& jittered,
                     int depth = 0);
    Vector3 shadePhong(Ray ray, RayHit hit, const Material& shade,
                       const std::vector<Vector2>& jittered, int depth);
    Vector3 shadeDielectric(Ray ray, RayHit hit, const Material& shade,
                            const std::vector<Vector2>& jittered, int depth);
    bool refractRay(Ray ray, Vector3 normal, Float index, Vector3& out);
    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit);
    bool testSphereRay(const Vector3& e, Float radius, Ray ray, Float t0, Float t1, RayHit& hit);
    bool testTriangleRay(const Triangle3& triangle, Ray ray, Float t0, Float t1, RayHit& hit,
                         bool singleSide);
    std::vector<Vector2> generateJittered(int n);
    RTCPUConfig conf;
    ThreadPool<Vector2> threadPool;
    Scene scene;
};
