#pragma once
#include <map>
#include <vector>
#include <variant>
#include <focg/common/geom.h>
#include <focg/common/linalg.h>
#include <focg/common/curve.h>
#include <focg/common/screen.h>
#include <focg/common/transform.h>

struct Ray {
    Vector3 origin;
    Vector3 dir;
};

struct Camera {
    Vector3 e;
    Basis basis;
    Float focal;
    
    Camera() = default;
    explicit Camera(Vector3 e, Basis basis, Float focal) : e(e), basis(basis), focal(focal) {
    }
    
    Ray generateRay(const Vector2& pos, const Screen& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth() - 0.5;
        const Float v = (pos.y() + 0.5) / screen.getHeight() - 0.5;
        const Vector3 dir = -1*focal* basis.w + u * basis.u + v * basis.v;
        return Ray { e , dir.normalized() };
    }
    
    Matrix VPOV(const Screen& screen) {
        const Matrix v = viewMatrix(basis, e);
        const Matrix p = perspectiveProjectionMatrix(-1.0, -2.0);
        const Matrix o = orthProjectionMatrix(-0.5, 0.5, -0.5, 0.5, -0.5, -2.0);
        const Matrix vp = viewportMatrix(screen.getWidth(), screen.getHeight());
        return vp*o*p*v;
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
