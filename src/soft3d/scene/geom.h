#pragma once
#include <soft3d/image/texture.h>
#include <soft3d/math/curve.h>
#include <soft3d/math/linalg.h>
#include <soft3d/scene/ray.h>

#include <optional>
#include <variant>

struct LambertianBRDF {
};

struct SpecularBRDF {
    Float R0;
};

struct CoupledBRDF {
    Float R0{ 0.1f};
    Float roughness{ 0.5f };
};

struct AntPhongBRDF {
    Float Rs;
    Float nu;
    Float nv;
};

using BRDF = std::variant<SpecularBRDF, CoupledBRDF, LambertianBRDF, AntPhongBRDF>;

struct Material {
    Vector3 diffuse;
    Vector3 specular;
    Float phong{ 100.0 };
    bool ignoreShadow{ false };
    BRDF brdf = LambertianBRDF{};
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
    TextureId particleMap{ 0 };

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

    const Material& material() const {
        return std::visit([](auto& data) { return data.material; }, data);
    }

    BoundingRect boundingRect() const {
        BoundingRect out;
        std::visit(
            overloaded{ [&](const PlainSphere& sphere) {
                           out.min = sphere.center - sphere.radius;
                           out.max = sphere.center + sphere.radius;
                       },
                        [&](const Sphere& sphere) {
                           out.min = sphere.center - sphere.radius;
                           out.max = sphere.center + sphere.radius;
                        },
                        [&](const PlainTriangle& triangle) {
                            out.min[0] =  std::min({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                  triangle.curve.pC.x() });
                            out.min[1] = std::min({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                  triangle.curve.pC.y() });
                            out.min[2] = std::min({ triangle.curve.pA.z(), triangle.curve.pB.z(),
                                                  triangle.curve.pC.z() });
                            out.max[0] = std::max({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                  triangle.curve.pC.x() });
                            out.max[1] = std::max({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                  triangle.curve.pC.y() });
                            out.max[2] = std::max({ triangle.curve.pA.z(), triangle.curve.pB.z(),
                                                  triangle.curve.pC.z() });
                        },
                        [&](const Triangle& triangle) {
                            out.min[0] = std::min({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                  triangle.curve.pC.x() });
                            out.min[1] = std::min({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                  triangle.curve.pC.y() });
                            out.min[2] = std::min({ triangle.curve.pA.z(), triangle.curve.pB.z(),
                                                  triangle.curve.pC.z() });
                            out.max[0] = std::max({ triangle.curve.pA.x(), triangle.curve.pB.x(),
                                                  triangle.curve.pC.x() });
                            out.max[1] = std::max({ triangle.curve.pA.y(), triangle.curve.pB.y(),
                                                  triangle.curve.pC.y() });
                            out.max[2] = std::max({ triangle.curve.pA.z(), triangle.curve.pB.z(),
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
