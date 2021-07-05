#pragma once

#include <soft3d/backend/interface.h>
#include <soft3d/common/scene.h>
#include <soft3d/common/threadpool.h>

struct RayHit {
    Vector3 normal;
    Vector3 pos;
    Float time;
    const Geometry* geom;
};

struct RTCPURenderer : public Renderer {
    RTCPURenderer();
    ~RTCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void renderPixel(const Vector2& pos, Image& screen);
    Vector3 rayColor(Ray ray, Float t0, Float t1, int depth = 0);
    Vector3 shadePlain(Ray ray, RayHit hit, const Shade& shade, int depth);

    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit);
    bool testSphereRay(const Vector3& e, Float radius, Ray ray, Float t0, Float t1, RayHit& hit);
    bool testTriangleRay(const Triangle3& triangle, Ray ray, Float t0, Float t1, RayHit& hit);

    ThreadPool<Vector2> threadPool;

    Scene scene;
};
