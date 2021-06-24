#pragma once
#include <focg/geom.h>
#include <focg/curve.h>
#include <focg/screen.h>

struct Basis {
    Vector3 u;
    Vector3 v;
    Vector3 w;
};


struct Ray {
    Vector3 origin;
    Vector3 dir;
};


struct Camera {
    Vector3 e;
    Basis basis;
    
    Camera() = default;
    explicit Camera(Vector3 e, Basis basis) : e(e), basis(basis) { }
    
    Ray generateRay(const Vector2& pos, const Screen& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth();
        const Float v = (pos.y() + 0.5) / screen.getHeight();
        const Float d = e.z();
        const Vector3 dir = -d * basis.w + u * basis.u + v * basis.v;
        return Ray { e , dir };
    }
};

// p(t) = e + t d
// f(p(t)) = 0
// (e+td - c)^2 - r^2 = 0
// d.d t^2 + 2d . (e-c)t + (e-c).(e-c) - r^2 = 0
// solve for t
// At^2 + Bt + C = 0
// A = d.d t^2
// B = 2d.(e-c)t
// C = (e-c).(e-c)
bool testRay(Ray ray, Sphere sphere, Vector3& normal) {
    Vector3 ec = ray.origin - sphere.c;
    Float dec = ray.dir.dot(ec);
    Float dd = ray.dir.dot(ray.dir);
    Float ecec = ec.dot(ec);
    Float D = dec * dec - dd*(ecec - sphere.r * sphere.r);
    
    const bool test = nearGt(D, 0.0);
    if (test) {
        Float t = (-dec + sqrt(D))/dd; // use -sqrt(D) solution that should be the earlier hit
        Vector3 p = ray.origin + t * ray.dir;
        normal = sphere.normal(p);
    }
    return test;
}

