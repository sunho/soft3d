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
    size_t pathSampleNum{ 10};
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
    Vector3 rayColor(Ray ray, Vector2 sample);
    Vector3 sampleLight(Geometry* geom, const Intersection& ins, const Vector3& pos, const Vector3& normal, const Vector3& ko, const Basis& TBN); 
    Vector3 sampleMediumLight(const Ray& ray, Medium* medium, const Vector3& pos); 
    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit);
    
    RTCPUConfig conf;
    ThreadPool<Vector2> threadPool;
    Scene scene;
    std::atomic<int> processedRays{ 0 };
};
