#pragma once
#include <focg/common/linalg.h>
#include <variant>

struct Shade {
    Shade() = default;
    Vector3 diffuse;
    Vector3 ambient;
    Vector3 specular;
    Vector3 reflect;
    Float phong{100.0};
};

struct Sphere {
    Vector3 center;
    Float radius {0.0};
    Shade shade;

    Sphere() = default;
    explicit Sphere(Vector3 center, Float radius, Shade shade) : center(center), radius(radius), shade(shade) { }
    
    // f(p) = (p-c)^2 - r^2 = 0
    Float test(const Vector3& p) {
        Vector3 t = center - p;
        return t.dot(t) - radius*radius;
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
    Vector3 normal(const Vector3& p) {
        return (p - center) / radius;
    }
};

struct Triangle {
    Vector3 vA;
    Vector3 vB;
    Vector3 vC;
    Shade shade;

    Triangle() = default;
    explicit Triangle(Vector3 a, Vector3 b, Vector3 c, Shade shade) : vA(a), vB(b), vC(c), shade(shade) {
        precompute();
    }

    void precompute() {
        Vector3 ab = vB-vA;
        Vector3 ac = vC-vA;
        n = ab.cross(ac);
        //printf("%s %s %s", ab.desc().c_str(), ac.desc().c_str(), n.desc().c_str());
        cA = vC-vB;
        cB = vA-vC;
        cC = vB-vA;
    }

    // The barycentric coordinates are proportional to the areas of the three subtriangles.
    // a = A_a / A
    // A_a = |(c-b)x(p - b)| / 2
    // A = |(b-a)x(c-a)| / 2
    // (b-a)x(c-a) = normal (it's perpendicular to the surface)
    // Note that signity of bc x bp and ab x ac is is same when p is inside the triangle
    // Use dot product to test this and normalize it cleverly
    // a = n.(c-b)x(p - b) / |n|^2
    // = |n| * |(c-b)x(p-b)| * cos(theta) / |n|^2
    // = |(c-b)x(p-b)| / |n|
    // = A_a / A
    // cos(theta) = 1 when it's inside the triangle
    // Resulting formulas
    // n = (b-a)x(c-a)
    // n_a = (c-b)x(p-b)
    // n_b = (a-c)x(p-c)
    // n_c = (b-a)x(p-a)
    // a = n.n_a/|n|^2
    // b = n.n_b/|n|^2
    // c = n.n_c/|n|^2
    Vector3 test(const Vector3& p) {
        Vector3 nA = cA.cross(p-vB);
        Vector3 nB = cB.cross(p-vC);
        Vector3 nC = cC.cross(p-vA);
        Float n2 = n.norm2();
        Float a = n.dot(nA) / n2;
        Float b = n.dot(nB) / n2;
        Float c = n.dot(nC) / n2;
        return Vector3(a,b,c);
    }
    
    Vector3 normal(const Vector3& p) {
        return n.normalized();
    }
private:
    Vector3 n;
    Vector3 cA;
    Vector3 cB;
    Vector3 cC;
};

using Geometry = std::variant<Sphere, Triangle>;
