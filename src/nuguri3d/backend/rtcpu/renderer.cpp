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
    screen.setPixel(pos, rayColor(ray, 0.0, 1.0f / 0.0f));
}

Vector3 RTCPURenderer::rayColor(Ray ray, Float t0, Float t1, int depth) {
    if (depth == 10) {
        return Vector3(0, 0, 0);
    }
    RayHit hit;
    const bool test = testRay(ray, t0, t1, hit);
    if (test) {
        Vector3 pixel;
        if (auto sphere = std::get_if<PlainSphere>(hit.geom)) {
            pixel = shadePlain(ray, hit, sphere->shade, depth);
        } else if (auto triangle = std::get_if<PlainTriangle>(hit.geom)) {
            pixel = shadePlain(ray, hit, triangle->shade, depth);
        } else if (auto sphere = std::get_if<Sphere>(hit.geom)) {
            Shade shade = sphere->shade;
            Vector3 color =
                scene.getTexture(sphere->texture)->lookupClamp(convertSphereTexcoord(hit.pos));
            shade.diffuse = color;
            shade.ambient = color;
            pixel = shadePlain(ray, hit, shade, depth);
        }
        return pixel;
    } else {
        return Vector3(0, 0, 0);
    }
}

Vector3 RTCPURenderer::shadePlain(Ray ray, RayHit hit, const Shade& shade, int depth) {
    StdLightSystem& lightSystem = std::get<StdLightSystem>(scene.lightSystem);
    Vector3 pixel = lightSystem.ambientIntensity * shade.ambient;
    RayHit hit2;
    for (auto light : lightSystem.lights) {
        if (!testRay(Ray{ hit.pos, light.v }, 0.0001f, 1.0f / 0.0f, hit2)) {
            Vector3 h = -1 * ray.dir.normalized() + light.v;
            h.normalize();

            Float x = std::max(0.0f, light.v.dot(hit.normal));
            Float x2 = std::max(0.0f, h.dot(hit.normal));
            pixel += light.intensity * (x * shade.diffuse + pow(x2, shade.phong) * shade.specular);
        }
    }

    Vector3 dir = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
    pixel += shade.reflect * rayColor(Ray{ hit.pos, dir }, 0.0001f, 1.0f / 0.0f, depth + 1);
    return pixel;
}

bool RTCPURenderer::testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
    bool out = false;
    for (auto& geom : scene.geoms) {
        bool tmp = false;
        if (auto sphere = std::get_if<PlainSphere>(&geom)) {
            tmp = testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        } else if (auto triangle = std::get_if<PlainTriangle>(&geom)) {
            tmp = testTriangleRay(*triangle, ray, t0, t1, hit);
        } else if (auto sphere = std::get_if<Sphere>(&geom)) {
            tmp = testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        }
        out |= tmp;
        if (tmp) {
            hit.geom = &geom;
            t1 = hit.time;
        }
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
bool RTCPURenderer::testSphereRay(const Vector3& e, Float radius, Ray ray, Float t0, Float t1,
                                  RayHit& hit) {
    Vector3 ec = ray.origin - e;
    Float dec = ray.dir.dot(ec);
    Float dd = ray.dir.dot(ray.dir);
    Float ecec = ec.dot(ec);
    Float D = dec * dec - dd * (ecec - radius * radius);
    bool test = nearGte(D, 0.0f);
    if (test) {
        const Float t =
            (-dec - sqrt(D)) / dd;  // use -sqrt(D) solution that should be the earlier hit
        if (inRange(t, t0, t1)) {
            hit.pos = ray.origin + t * ray.dir;
            hit.time = t;

            // Normal calculation
            // grad f(p)
            // (p-c)^2 = p^2 - 2p.c + C
            // = p_x^2 + p_y^2 + p_z^2 - 2p_xc_x - 2p_yc_y - 2p_zc_z + C
            //
            // del x = 2p_x - 2c_x del y = 2p_y - 2c_y del z = 2p_z - 2c_z
            // grad = 2(p-c)
            // on the surface, |(p-c)| = r (from solving f(p))
            // |2(p-c)| = 2|p-c| = 2r
            // unit grad = (p-c)/r
            hit.normal = ((hit.pos - e) / radius).normalized();
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
bool RTCPURenderer::testTriangleRay(const PlainTriangle& triangle, Ray ray, Float t0, Float t1,
                                    RayHit& hit) {
    Vector3 vA = triangle.vA.transformed(triangle.transform, 1.0f);
    Vector3 vB = triangle.vB.transformed(triangle.transform, 1.0f);
    Vector3 vC = triangle.vC.transformed(triangle.transform, 1.0f);
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
    if (!nearInRange(t, t0, t1)) {
        return false;
    }
    Float gamma = (i * (a * k - j * b) + h * (j * c - a * l) + g * (b * l - k * c)) / M;
    if (!nearInRange(gamma, 0.0f, 1.0f)) {
        return false;
    }
    Float beta = (j * (e * i - h * f) + k * (g * f - d * i) + l * (d * h - e * g)) / M;
    if (!nearInRange(beta, 0.0f, 1.0f - gamma)) {
        return false;
    }
    hit.pos = ray.origin + t * ray.dir;
    hit.time = t;
    Vector3 ab = vB - vA;
    Vector3 ac = vC - vA;
    hit.normal = ab.cross(ac).normalized();
    return true;
}
