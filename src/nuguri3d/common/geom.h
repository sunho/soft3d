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

    // f(p) = (p-c)^2 - r^2 = 0
    Float test(const Vector3& p) {
        Vector3 t = center - p;
        return t.dot(t) - radius * radius;
    }

    // grad f(p)
    // (p-c)^2 = p^2 - 2p.c + C
    // = p_x^2 + p_y^2 + p_z^2 - 2p_xc_x - 2p_yc_y - 2p_zc_z + C
    //
    // del x = 2p_x - 2c_x del y = 2p_y - 2c_y del z = 2p_z - 2c_z
    // grad = 2(p-c)
    // on the surface, |(p-c)| = r (from solving f(p))
    // |2(p-c)| = 2|p-c| = 2r
    // unit grad = (p-c)/r
    Vector3 normal(const Vector3& p) const {
        return (p - center) / radius;
    }
};

struct Sphere {
    Vector3 center;
    Float radius{ 0.0 };
    Shade shade;
    Texture* texture;

    Sphere() = default;
    explicit Sphere(Vector3 center, Float radius, Shade shade, Texture* texture)
        : center(center), radius(radius), shade(shade), texture(texture) {
    }

    Float test(const Vector3& p) {
        Vector3 t = center - p;
        return t.dot(t) - radius * radius;
    }

    Vector3 normal(const Vector3& p) const {
        return (p - center) / radius;
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
    Vector3 tex;
};

struct Triangle {
    TriangleVertex vA;
    TriangleVertex vB;
    TriangleVertex vC;
    Triangle3 curve;
    Shade shade;
    Texture* texture{ nullptr };

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
