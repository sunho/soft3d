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
    size_t pathSampleNum{ 50 };
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
    bool refractRay(Ray ray, Vector3 normal, Float index, Vector3& out);
    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit);
    bool testRayAndFetchTex(Ray ray, Float t0, Float t1, RayHit& hit, Material& material);
    bool testSphereRay(const Vector3& e, Float radius, Ray ray, Float t0, Float t1, RayHit& hit);
    bool testTriangleRay(const Triangle3& triangle, Ray ray, Float t0, Float t1, RayHit& hit,
                         bool singleSide);
    
    Vector3 sampleBRDF(const Material& material, const Vector3& ko, Vector3& ki, Float& pdf, bool& specular);
    Vector3 sampleLight(const Material& material, const Vector3& pos, const Vector3& normal,
                        const Vector3& ko, const Basis& TBN);
    Vector3 evalBRDF(const Material& material, const Vector3& ko, const Vector3& ki);
    Float pdfBRDF(const Material& material, const Vector3& ki);
    Vector3 sampleHemisphere(const Vector2& sample);
    

    std::vector<Vector2> generateJittered(int n);
    RTCPUConfig conf;
    ThreadPool<Vector2> threadPool;
    Scene scene;
    std::atomic<int> processedRays{ 0 };
};
