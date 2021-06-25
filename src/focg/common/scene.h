#pragma once
#include <map>
#include <vector>
#include <variant>
#include <focg/common/geom.h>
#include <focg/common/linalg.h>
#include <focg/common/curve.h>
#include <focg/common/screen.h>

struct Ray {
    Vector3 origin;
    Vector3 dir;
};

struct Camera {
    Vector3 e;
    Basis basis;
    
    Camera() = default;
    explicit Camera(Vector3 e, Basis basis) : e(e), basis(basis) {
    }
    
    Ray generateRay(const Vector2& pos, const Screen& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth();
        const Float v = (pos.y() + 0.5) / screen.getHeight();
        const Float d = e.z();
        const Vector3 dir = -1*d * basis.w + u * basis.u + v * basis.v;
        return Ray { e , dir.normalized() };
    }
};

struct UniformLight {
    Float intensity;
    Vector3 v;
};

struct StdLightSystem {
    Float ambientIntensity;
    std::vector<UniformLight> lights;
};

using LightSystem = std::variant<StdLightSystem>;

struct Scene {
    Scene() = default;
    
    std::vector<Geometry> geoms;
    Camera camera;
    LightSystem lightSystem;
};
