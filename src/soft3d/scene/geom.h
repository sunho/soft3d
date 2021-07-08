#pragma once
#include <soft3d/image/texture.h>
#include <soft3d/math/curve.h>
#include <soft3d/math/linalg.h>
#include <soft3d/scene/ray.h>

#include <optional>
#include <variant>

struct Material {
    Material() = default;
    Vector3 diffuse;
    Vector3 ambient;
    Vector3 specular;
    Float phong{ 100.0 };
    std::optional<Vector3> idealReflect{ std::nullopt };
    std::optional<Float> refractIndex{ std::nullopt };
    Vector3 refractReflectance;
    bool ignoreShadow{ false };
};

struct PlainSphere {
    Vector3 center;
    Float radius{ 0.0 };
    Material material;

    PlainSphere() = default;
    explicit PlainSphere(Vector3 center, Float radius, Material material)
        : center(center), radius(radius), material(material) {
    }
};

struct Sphere {
    Vector3 center;
    Float radius{ 0.0 };
    Material material;
    TextureId texture{ 0 };

    Sphere() = default;
    explicit Sphere(Vector3 center, Float radius, Material material, TextureId texture)
        : center(center), radius(radius), material(material), texture(texture) {
    }
};

struct PlainTriangle {
    Vector3 vA;
    Vector3 vB;
    Vector3 vC;
    Triangle3 curve;
    Material material;

    PlainTriangle() = default;
    explicit PlainTriangle(Vector3 a, Vector3 b, Vector3 c, Material material)
        : vA(a), vB(b), vC(c), material(material), curve(a, b, c) {
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
    Material material;
    TextureId texture{ 0 };
    TextureId normalMap{ 0 };

    Triangle() = default;
    explicit Triangle(TriangleVertex a, TriangleVertex b, TriangleVertex c, Material material)
        : vA(a), vB(b), vC(c), material(material), curve(a.pos, b.pos, c.pos) {
    }

    Vector3 normal(const Vector3& p) const {
        const Vector3 bary = curve(p);
        return bary.x() * vA.normal + bary.y() * vB.normal + bary.z() * vC.normal;
    }
};

using GeometryData = std::variant<PlainSphere, PlainTriangle, Triangle, Sphere>;

struct Geometry {
    Geometry() = default;
    Geometry(GeometryData&& data) : data(data) {
    }

    Material material() const {
        return std::visit([](auto& data) { return data.material; }, data);
    }

    BoundingRect boundingRect() const {
        BoundingRect out;
        std::visit(
            overloaded{ [&](const PlainSphere& sphere) {
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
            data);
        return out;
    }

    template <typename T>
    T* get() {
        return std::get_if<T>(&data);
    }

    template <typename T>
    const T* get() const {
        return std::get_if<T>(&data);
    }

  private:
    GeometryData data;
};
