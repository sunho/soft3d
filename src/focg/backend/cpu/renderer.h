#pragma once

#include <focg/common/scene.h>
#include <focg/backend/interface.h>

struct RayHit {
    Vector3 normal;
    Vector3 pos;
    Float time;
    Shade shade;
};

struct CPURenderer : public Renderer {
    CPURenderer();
    ~CPURenderer();
    
    Scene& sceneRef() override;
    void render(Screen& screen) override;

private:
    void renderPixel(const Vector2& pos, Screen& screen);
    Vector3 rayColor(Ray ray, Float t0, Float t1, int depth = 0);

    bool testRay(Ray ray, Float t0, Float t1, RayHit &hit);
    bool testSphereRay(Sphere sphere, Ray ray, Float t0, Float t1, RayHit &hit);
    bool testTriangleRay(Triangle triangle, Ray ray, Float t0, Float t1, RayHit &hit);
    
    Scene scene;
};
