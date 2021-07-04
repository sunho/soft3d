#pragma once
#include <nuguri3d/common/linalg.h>
#include <nuguri3d/common/texture.h>

#include <variant>

struct Shade {
    Shade() = default;
    Vector3 diffuse;
    Vector3 ambient;
    Vector3 specular;
    Vector3 reflect;
    Float phong{ 100.0 };
};

struct PlainSphere {
    Matrix transform{ I4x4 };
    Matrix itransform{ I4x4 };
    Vector3 center;
    Float radius{ 0.0 };
    Shade shade;

    PlainSphere() = default;
    explicit PlainSphere(Vector3 center, Float radius, Shade shade)
        : center(center), radius(radius), shade(shade) {
    }
};

struct Sphere {
    Vector3 center;
    Float radius{ 0.0 };
    Shade shade;
    TextureId texture{ 0 };

    Sphere() = default;
    explicit Sphere(Vector3 center, Float radius, Shade shade, TextureId texture)
        : center(center), radius(radius), shade(shade), texture(texture) {
    }
};

struct PlainTriangle {
    Matrix transform{ I4x4 };
    Vector3 vA;
    Vector3 vB;
    Vector3 vC;
    Triangle3 curve;
    Shade shade;

    PlainTriangle() = default;
    explicit PlainTriangle(Vector3 a, Vector3 b, Vector3 c, Shade shade)
        : vA(a), vB(b), vC(c), shade(shade), curve(a, b, c) {
    }

    Vector3 normal(const Vector3& p) const {
        return curve.n;
    }
};

struct TriangleVertex {
    Vector3 pos;
    Vector3 normal;
    Vector2 tex;
};

struct Triangle {
    TriangleVertex vA;
    TriangleVertex vB;
    TriangleVertex vC;
    Triangle3 curve;
    Shade shade;
    TextureId texture{ 0 };

    Triangle() = default;
    explicit Triangle(TriangleVertex a, TriangleVertex b, TriangleVertex c, Shade shade)
        : vA(a), vB(b), vC(c), shade(shade), curve(a.pos, b.pos, c.pos) {
    }

    Vector3 normal(const Vector3& p) const {
        const Vector3 bary = curve(p);
        return bary.x() * vA.normal + bary.y() * vB.normal + bary.z() * vC.normal;
    }
};

using Geometry = std::variant<PlainSphere, PlainTriangle, Triangle, Sphere>;
