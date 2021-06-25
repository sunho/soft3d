#pragma once
#include <map>
#include <vector>
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
    explicit Camera(Vector3 e, Basis basis) : e(e), basis(basis) {
    }
    
    Ray generateRay(const Vector2& pos, const Screen& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth();
        const Float v = (pos.y() + 0.5) / screen.getHeight();
        const Float d = e.z();
        const Vector3 dir = -1*d * basis.w + u * basis.u + v * basis.v;
        return Ray { e , dir.normalized() };
    }
};

struct Shade {
    Vector3 diffuse;
    Vector3 ambient;
    Vector3 specular;
    Float p;
};

struct RayObject;

struct RayHit {
    Vector3 normal;
    Vector3 pos;
    Float time;
    Shade shade;
    RayObject* obj;
};

enum class RayObjectType {
    SPHERE,
    TRIANGLE3
};

struct RayObject {
    RayObject() = default;
    virtual ~RayObject() = default;
    RayObject(const RayObject&) = default;
    RayObject& operator =(const RayObject&) = default;
    RayObject(RayObject&&) = default;
    RayObject& operator =(RayObject&&) = default;
    
    virtual bool testRay(Ray ray, Float t0, Float t1, RayHit& hit) = 0;
    virtual RayObjectType type() = 0;
};

struct Sphere : public RayObject {
    Vector3 center;
    Float radius {0.0};
    Shade shade;

    Sphere() = default;
    ~Sphere() { }
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

    // p(t) = e + t d
    // f(p(t)) = 0
    // (e+td - c)^2 - r^2 = 0
    // d.d t^2 + 2d . (e-c)t + (e-c).(e-c) - r^2 = 0
    // solve for t
    // At^2 + Bt + C = 0
    // A = d.d t^2
    // B = 2d.(e-c)t
    // C = (e-c).(e-c)
    bool testRay(Ray ray, Float t0, Float t1, RayHit &hit) override {
        Vector3 ec = ray.origin - center;
        Float dec = ray.dir.dot(ec);
        Float dd = ray.dir.dot(ray.dir);
        Float ecec = ec.dot(ec);
        Float D = dec * dec - dd*(ecec - radius * radius);
        bool test = nearGt(D, 0.0);
        if (test) {
            const Float t = (-dec - sqrt(D))/dd; // use -sqrt(D) solution that should be the earlier hit
            if (inRange(t, t0, t1)) {
                hit.pos = ray.origin + t * ray.dir;
                hit.time = t;
                hit.normal = normal(hit.pos);
                hit.shade = shade;
                hit.obj = this;
            } else {
                test = false;
            }
        }
        return test;
    }
    
    RayObjectType type() override {
        return RayObjectType::SPHERE;
    }
};

struct Triangle3 : public RayObject {
    Vector3 vA;
    Vector3 vB;
    Vector3 vC;
    Shade shade;

    Triangle3() = default;
    ~Triangle3() { }
    explicit Triangle3(Vector3 a, Vector3 b, Vector3 c, Shade shade) : vA(a), vB(b), vC(c), shade(shade) {
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
    
    // Just solve this equation:
    // e + td = a + x(b-a) + y(c-a)
    // solve for t, x, y
    // it will give linear system with A = 3x3 -> can be full column rank (is it always full rank?)
    // a = x_a - x_b
    // b = y_a - y_b
    // c = z_a - z_b
    // d = x_a - x_c
    // e = y_a - y_c
    // f = z_a - z_c
    // g = x_d
    // h = y_d
    // i = z_d
    // j = x_a - x_e
    // k = y_a - y_e
    // i = z_a - z_e
    //
    // b = j(ei-hf)+k(gf-di)+l(dh-eg) / M
    // c = i(ak-jb)+h(jc-al)+g(bl-kc) / M
    // t = -(f(ak-jb)+e(jc-al)+d(bl-kc) / M)
    // M = a(ei-hf) + b(gf-di) + c(dh-eg)
    bool testRay(Ray ray, Float t0, Float t1, RayHit &hit) override {
        Float a = vA.x() - vB.x();
        Float b = vA.y() - vB.y();
        Float c = vA.z() - vB.z();
        Float d = vA.x() - vC.x();
        Float e = vA.y() - vC.y();
        Float f = vA.z() - vC.z();
        Float g = ray.dir.x();
        Float h = ray.dir.y();
        Float i = ray.dir.z();
        Float j = vA.x() - ray.origin.x();
        Float k = vA.y() - ray.origin.y();
        Float l = vA.z() - ray.origin.z();
        
        // I believe in compiler
        Float M = a*(e*i-h*f) + b*(g*f-d*i) + c*(d*h-e*g);
        Float t = -((f*(a*k-j*b)+e*(j*c-a*l)+d*(b*l-k*c)) / M);
        //printf("t: %f \n", t);
        if (!inRange(t,t0,t1)) {
            return false;
        }
        Float gamma = (i*(a*k-j*b)+h*(j*c-a*l)+g*(b*l-k*c)) / M;
        //printf("g: %f \n", gamma);
        if (!inRange(gamma, 0.0, 1.0)) {
            return false;
        }
        Float beta =(j*(e*i-h*f)+k*(g*f-d*i)+l*(d*h-e*g)) / M;
        //printf("b: %f \n", beta);
        if (!inRange(beta, 0.0, 1.0 - gamma)) {
            return false;
        }
        hit.pos = ray.origin + t * ray.dir;
        hit.time = t;
        hit.normal = n.normalized();
        hit.shade = shade;
        hit.obj = this;
        return true;
    }
    
    RayObjectType type() override {
        return RayObjectType::TRIANGLE3;
    }
private:
    Vector3 n;
    Vector3 cA;
    Vector3 cB;
    Vector3 cC;
};

using RayObjectPtr = std::unique_ptr<RayObject>;

struct Scene {
    Scene() = default;
    Scene(const Scene&) = delete;
    Scene & operator=(const Scene&) = delete;
    
    Scene(Scene &&o) noexcept {
        std::swap(objects, o.objects);
    }

    Scene &operator=(Scene &&o) noexcept {
        std::swap(objects, o.objects);
        return *this;
    }
    
    void addObject(int order, RayObjectPtr obj) {
        objects.emplace(order, std::move(obj));
    }
    
    bool testRay(Ray ray, Float t0, Float t1, RayHit &hit) {
        bool test = false;
        for (auto& [_, obj] : objects) {
            if (obj->testRay(ray, t0, t1, hit)) {
                t1 = hit.time;
                test = true;
            }
        }
        return test;
    }
private:
    std::map<int, RayObjectPtr> objects;
};
