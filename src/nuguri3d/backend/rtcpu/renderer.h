#pragma once

#include <nuguri3d/backend/interface.h>
#include <nuguri3d/common/scene.h>

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
    bool testSphereRay(const PlainSphere& sphere, Ray ray, Float t0, Float t1, RayHit& hit);
    bool testSphereRay2(const Sphere& sphere, Ray ray, Float t0, Float t1, RayHit& hit);
    bool testTriangleRay(const PlainTriangle& triangle, Ray ray, Float t0, Float t1, RayHit& hit);

    Scene scene;
};
