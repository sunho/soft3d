#pragma once

#include <focg/backend/interface.h>
#include <focg/common/scene.h>

struct RayHit {
    Vector3 normal;
    Vector3 pos;
    Float time;
    Shade shade;
};

struct RTCPURenderer : public Renderer {
    RTCPURenderer();
    ~RTCPURenderer();

    Scene& sceneRef() override;
    void render(Image& screen) override;

  private:
    void renderPixel(const Vector2& pos, Image& screen);
    Vector3 rayColor(Ray ray, Float t0, Float t1, int depth = 0);

    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit);
    bool testSphereRay(const Sphere& sphere, Ray ray, Float t0, Float t1, RayHit& hit);
    bool testTriangleRay(const Triangle& triangle, Ray ray, Float t0, Float t1, RayHit& hit);

    Scene scene;
};
