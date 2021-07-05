#pragma once
#include <soft3d/common/curve.h>
#include <soft3d/common/linalg.h>
#include <soft3d/common/texture.h>

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
        return curve.n.normalized();
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
    TextureId normalMap{ 0 };

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

struct Ray {
    Vector3 origin;
    Vector3 dir;
};

struct RayHit {
    Vector3 normal;
    Vector3 pos;
    Float time;
    const Geometry* geom;
};

struct BoundingRect {
    BoundingRect() = default;

    Float minX{ 0.0f };
    Float minY{ 0.0f };
    Float minZ{ 0.0f };
    Float maxX{ 0.0f };
    Float maxY{ 0.0f };
    Float maxZ{ 0.0f };

    // Assuming ray.dir.x y z >= 1
    // t_xmin = x_min - x_e / x_d
    // ...
    // the ray hits iff [t_xmin, t_xmax], [t_ymin, t_ymax], and [t_zmin, y_zmax] overlaps
    // for x_d < 0, t_xmin and t_xmax flip
    // fox x_d = 0 it's more subtle
    // ray hit iff x_min < x_e < x_max
    // no hit when x_e <= x_min
    // or x_e >= x_max
    // for first case: t_max = inf t_min = inf (inf,inf) -> no hit
    // for hit case: t_min = -inf t_max = inf (-inf,inf) -> every hit
    // for NaN -> conditions will be evaluated as false <- this case is just taken as hit actually
    // as IEEE rule
    bool hit(Ray ray) const {
        Float facx = 1.0 / ray.dir.x();
        Float tminX = (minX - ray.origin.x()) * facx;
        Float tmaxX = (maxX - ray.origin.x()) * facx;
        if (ray.dir.x() < 0) {
            std::swap(tminX, tmaxX);
        }
        Float facy = 1.0 / ray.dir.y();
        Float tminY = (minY - ray.origin.y()) * facy;
        Float tmaxY = (maxY - ray.origin.y()) * facy;
        if (ray.dir.y() < 0) {
            std::swap(tminY, tmaxY);
        }
        Float facz = 1.0 / ray.dir.z();
        Float tminZ = (minZ - ray.origin.z()) * facz;
        Float tmaxZ = (maxZ - ray.origin.z()) * facz;
        if (ray.dir.z() < 0) {
            std::swap(tminZ, tmaxZ);
        }
        if (tminX > tmaxY || tmaxX < tminY || tminX > tmaxZ || tmaxX < tminZ || tminY > tmaxZ ||
            tmaxY < tminZ) {
            return false;
        } else {
            return true;
        }
    }

    BoundingRect operator+(const BoundingRect& other) const {
        return BoundingRect{ std::min(minX, other.minX), std::min(minY, other.minY),
                             std::min(minZ, other.minZ), std::max(maxX, other.maxX),
                             std::max(maxY, other.maxY), std::max(maxZ, other.maxZ) };
    }

    Float minComp(int axis) const {
        if (axis == 0) {
            return minX;
        } else if (axis == 1) {
            return minY;
        } else {
            return minZ;
        }
    }

    Vector3 mid() const {
        return Vector3((minX + maxX) / 2.0f, (minY + maxY) / 2.0f, (minZ + maxZ) / 2.0f);
    }
};

static BoundingRect getBoundingRect(const Geometry& geom) {
    BoundingRect out;
    std::visit(overloaded{ [&](const PlainSphere& sphere) {
                              out.minX = sphere.center.x() - sphere.radius;
                              out.minY = sphere.center.y() - sphere.radius;
                              out.minZ = sphere.center.z() - sphere.radius;
                              out.maxX = sphere.center.x() + sphere.radius;
                              out.maxY = sphere.center.y() + sphere.radius;
                              out.maxZ = sphere.center.z() + sphere.radius;
                          },
                           [&](const Sphere& sphere) {
                               out.minX = sphere.center.x() - sphere.radius;
                               out.minY = sphere.center.y() - sphere.radius;
                               out.minZ = sphere.center.z() - sphere.radius;
                               out.maxX = sphere.center.x() + sphere.radius;
                               out.maxY = sphere.center.y() + sphere.radius;
                               out.maxZ = sphere.center.z() + sphere.radius;
                           },
                           [&](const PlainTriangle& triangle) {
                               out.minX = std::min({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                     triangle.curve.pC.x() });
                               out.minY = std::min({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                     triangle.curve.pC.y() });
                               out.minZ = std::min({ triangle.curve.pA.z(), triangle.curve.pB.z(),
                                                     triangle.curve.pC.z() });
                               out.maxX = std::max({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                     triangle.curve.pC.x() });
                               out.maxY = std::max({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                     triangle.curve.pC.y() });
                               out.maxZ = std::max({ triangle.curve.pA.z(), triangle.curve.pB.z(),
                                                     triangle.curve.pC.z() });
                           },
                           [&](const Triangle& triangle) {
                               out.minX = std::min({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                     triangle.curve.pC.x() });
                               out.minY = std::min({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                     triangle.curve.pC.y() });
                               out.minZ = std::min({ triangle.curve.pA.z(), triangle.curve.pB.z(),
                                                     triangle.curve.pC.z() });
                               out.maxX = std::max({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                     triangle.curve.pC.x() });
                               out.maxY = std::max({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                     triangle.curve.pC.y() });
                               out.maxZ = std::max({ triangle.curve.pA.z(), triangle.curve.pB.z(),
                                                     triangle.curve.pC.z() });
                           } },
               geom);
    return out;
}
