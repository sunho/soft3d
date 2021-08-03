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

struct Light {
    Light() = default;
    virtual ~Light() {
    }
    virtual Vector3 sampleDir(const Vector3& pos) = 0;
    virtual Vector3 Le(const Vector3& brdf, const Vector3& ki, const Vector3& dir) = 0;
    virtual Vector3 Le() = 0;
};

struct DirectionalLight : public Light {
    DirectionalLight() = default;
    ~DirectionalLight() {
    }
    DirectionalLight(Vector3 intensity, Vector3 v, Float R) : intensity(intensity), v(v), R(R) {
    }
    Vector3 sampleDir(const Vector3& pos) override {
        return -10000.0f * v; // hack
    }
    Vector3 Le(const Vector3& brdf, const Vector3& ki, const Vector3& dir) override {
        Float cosTh = clamp(Vector3(0,0,1).dot(ki), 0.0f, 1.0f);
        return intensity * cosTh * clamp(-dir.normalized().dot(v), 0.0f, 1.0f) * brdf;
    }

    Vector3 Le() override {
        return intensity;
    }
    Vector3 intensity;
    Vector3 v;
    Float R;
};

struct AreaLight : public Light {
    AreaLight() = default;
    ~AreaLight() {
    }
    AreaLight(Vector3 intensity, Vector3 pos, Vector3 edge1, Vector3 edge2)
        : intensity(intensity)
        , pos(pos)
        , edge1(edge1), edge2(edge2) {
    }

    Vector3 sampleDir(const Vector3& pos) override {
        Float u = randUniform();
        Float v = randUniform();
        Vector3 lightPos = this->pos + u * edge1 + v * edge2;
        return lightPos - pos;
    }

    Vector3 Le(const Vector3& brdf, const Vector3& ki, const Vector3& dir) override {
        Vector3 lightN = edge1.cross(edge2).normalized();
        Float cosTh = clamp(Vector3(0,0,1).dot(ki), 0.0f, 1.0f);
        Float cosThd = clamp(-lightN.dot(dir.normalized()), 0.0f, 1.0f);
        return intensity * cosTh * cosThd * brdf / dir.norm2();
    }

    Vector3 Le() override {
        return intensity;
    }

    Vector3 intensity;
    Vector3 pos;
    Vector3 edge1;
    Vector3 edge2;
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

