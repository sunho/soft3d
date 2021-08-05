#pragma once
#include <soft3d/common/idresource.h>
#include <soft3d/image/image.h>
#include <soft3d/image/texture.h>
#include <soft3d/math/curve.h>
#include <soft3d/math/linalg.h>
#include <soft3d/math/transform.h>
#include <soft3d/scene/bvhtree.h>
#include <soft3d/common/resourcelist.h>
#include <soft3d/scene/geom.h>

#include <list>
#include <map>
#include <set>
#include <variant>
#include <vector>

struct Camera {
    Vector3 e;
    Basis basis;
    Float focal;

    Camera() = default;
    explicit Camera(Vector3 e, Basis basis, Float focal) : e(e), basis(basis), focal(focal) {
    }

    Ray generateRay(const Vector2& pos, const Image& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth() - 0.5;
        const Float v = (pos.y() + 0.5) / screen.getHeight() - 0.5;
        const Vector3 dir = -1 * focal * basis.w + u * basis.u + v * basis.v;
        return Ray{ e, dir.normalized() };
    }

    Ray generateOrthRay(const Vector2& pos, const Image& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth() - 0.5;
        const Float v = (pos.y() + 0.5) / screen.getHeight() - 0.5;
        return Ray{ e + u * basis.u + v * basis.v, -1 * basis.w };
    }

    Matrix VPOV(const Image& screen) {
        const Matrix v = viewMatrix(basis, e);
        const Matrix p = perspectiveProjectionMatrix(-1.0, -2.0);
        const Matrix o = orthProjectionMatrix(-0.5, 0.5, -0.5, 0.5, -0.5, -2.0);
        const Matrix vp = viewportMatrix(screen.getWidth(), screen.getHeight());
        return vp * o * p * v;
    }

    void lookAt(Vector3 pos, Vector3 up) {
        Vector3 w = (e - pos).normalized();
        Vector3 u = up.cross(w).normalized();
        Vector3 v = w.cross(u);
        basis.u = u;
        basis.v = v;
        basis.w = w;
    }
};

struct Scene {
    Scene() = default;
    
    void prepare() {
        bvhtree = BvhTree();   
        for (auto geom : geoms.list()) {
            bvhtree.add(geom);
        }
        bvhtree.prepare();
    }

    ResourceList<Light> lights;
    ResourceList<Geometry> geoms;
    ResourceList<Image> textures;
    ResourceList<BRDF> brdfs;
    BvhTree bvhtree;
    Camera camera;
    Image* environmentMap{ nullptr };
};

