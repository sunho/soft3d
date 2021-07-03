#include "renderer.h"

RTCPURenderer::RTCPURenderer() {
}

RTCPURenderer::~RTCPURenderer() {
}

Scene& RTCPURenderer::sceneRef() {
    return scene;
}

void RTCPURenderer::render(Image& screen) {
    for (int i = 0; i < screen.getWidth(); ++i) {
        for (int j = 0; j < screen.getHeight(); ++j) {
            Vector2 pos(i, j);
            renderPixel(pos, screen);
        }
    }
}

void RTCPURenderer::renderPixel(const Vector2& pos, Image& screen) {
    const Ray ray = scene.camera.generateRay(pos, screen);
    screen.setPixel(pos, rayColor(ray, 0.0, 1.0 / 0.0));
}

Vector3 RTCPURenderer::rayColor(Ray ray, Float t0, Float t1, int depth) {
    if (depth == 10) {
        return Vector3(0, 0, 0);
    }
    RayHit hit;
    RayHit hit2;
    const bool test = testRay(ray, t0, t1, hit);
    StdLightSystem& lightSystem = std::get<StdLightSystem>(scene.lightSystem);
    if (test) {
        Vector3 pixel = lightSystem.ambientIntensity * hit.shade.ambient;

        for (auto light : lightSystem.lights) {
            if (!testRay(Ray{ hit.pos, light.v }, 0.0001, 1.0 / 0.0, hit2)) {
                Vector3 h = -1 * ray.dir.normalized() + light.v;
                h.normalize();

                Float x = std::max(0.0f, light.v.dot(hit.normal));
                Float x2 = std::max(0.0f, h.dot(hit.normal));
                pixel += light.intensity *
                         (x * hit.shade.diffuse + pow(x2, hit.shade.phong) * hit.shade.specular);
            }
        }

        Vector3 dir = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
        pixel += hit.shade.reflect * rayColor(Ray{ hit.pos, dir }, 0.0001, 1.0 / 0.0, depth + 1);

        return pixel;
    } else {
        return Vector3(0.0, 0.0, 0.0);
    }
}

bool RTCPURenderer::testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
    bool out = false;
    for (auto& geom : scene.geoms) {
        bool tmp = false;
        if (auto sphere = std::get_if<Sphere>(&geom)) {
            tmp = testSphereRay(*sphere, ray, t0, t1, hit);
        } else if (auto triangle = std::get_if<Triangle>(&geom)) {
            tmp = testTriangleRay(*triangle, ray, t0, t1, hit);
        }
        out |= tmp;
        if (tmp)
            t1 = hit.time;
    }
    return out;
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
bool RTCPURenderer::testSphereRay(const Sphere& sphere, Ray ray, Float t0, Float t1, RayHit& hit) {
    ray.origin = ray.origin.transformed(sphere.itransform, 1.0);
    ray.dir = ray.dir.transformed(sphere.itransform, 0.0);
    Vector3 ec = ray.origin - sphere.center;
    Float dec = ray.dir.dot(ec);
    Float dd = ray.dir.dot(ray.dir);
    Float ecec = ec.dot(ec);
    Float D = dec * dec - dd * (ecec - sphere.radius * sphere.radius);
    bool test = nearGte(D, 0.0);
    if (test) {
        const Float t =
            (-dec - sqrt(D)) / dd;  // use -sqrt(D) solution that should be the earlier hit
        if (inRange(t, t0, t1)) {
            hit.pos = (ray.origin + t * ray.dir).transformed(sphere.transform, 1.0);
            hit.time = t;
            hit.normal = sphere.normal(hit.pos).transformed(sphere.transform, 0.0).normalized();
            hit.shade = sphere.shade;
        } else {
            test = false;
        }
    }
    return test;
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
bool RTCPURenderer::testTriangleRay(const Triangle& triangle, Ray ray, Float t0, Float t1,
                                    RayHit& hit) {
    Vector3 vA = triangle.vA.transformed(triangle.transform, 1.0);
    Vector3 vB = triangle.vB.transformed(triangle.transform, 1.0);
    Vector3 vC = triangle.vC.transformed(triangle.transform, 1.0);
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
    Float M = a * (e * i - h * f) + b * (g * f - d * i) + c * (d * h - e * g);
    Float t = -((f * (a * k - j * b) + e * (j * c - a * l) + d * (b * l - k * c)) / M);
    // printf("t: %f \n", t);
    if (!nearInRange(t, t0, t1)) {
        return false;
    }
    Float gamma = (i * (a * k - j * b) + h * (j * c - a * l) + g * (b * l - k * c)) / M;
    // printf("g: %f \n", gamma);
    if (!nearInRange(gamma, 0.0, 1.0)) {
        return false;
    }
    Float beta = (j * (e * i - h * f) + k * (g * f - d * i) + l * (d * h - e * g)) / M;
    // printf("b: %f \n", beta);
    if (!nearInRange(beta, 0.0, 1.0 - gamma)) {
        return false;
    }
    hit.pos = ray.origin + t * ray.dir;
    hit.time = t;
    Vector3 ab = vB - vA;
    Vector3 ac = vC - vA;
    hit.normal = ab.cross(ac).normalized();
    hit.shade = triangle.shade;
    return true;
}
