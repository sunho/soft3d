#include "renderer.h"

constexpr int THREAD_NUM = 8;
constexpr int MAX_JOB = 2000 * 2000;
constexpr int SAMPLE_N = 3;
constexpr bool AAON = false;

RTCPURenderer::RTCPURenderer() : threadPool(THREAD_NUM, MAX_JOB) {
}

RTCPURenderer::~RTCPURenderer() {
}

Scene& RTCPURenderer::sceneRef() {
    return scene;
}

void RTCPURenderer::render(Image& screen) {
    threadPool.setJobFunc([&](Vector2 pos) { renderPixel(pos, screen); });

    for (int i = 0; i < screen.getWidth(); ++i) {
        for (int j = 0; j < screen.getHeight(); ++j) {
            Vector2 pos(i, j);
            threadPool.addJob(pos);
        }
    }
    threadPool.flush(THREAD_NUM);
}

void RTCPURenderer::renderPixel(const Vector2& pos, Image& screen) {
    auto jittered = generateJittered(SAMPLE_N);
    if (!AAON) {
        const Ray ray = scene.camera.generateRay(pos, screen);
        screen.setPixel(pos, rayColor(ray, 0.0, 1.0f / 0.0f, jittered));
    } else {
        Vector3 out;
        auto jittered2 = generateJittered(SAMPLE_N);
        for (auto& sample : jittered2) {
            Vector2 pos2 = pos + sample;
            const Ray ray = scene.camera.generateRay(pos2, screen);
            out += rayColor(ray, 0.0, 1.0f / 0.0f, jittered) / (SAMPLE_N * SAMPLE_N);
        }
        screen.setPixel(pos, out);
    }
}

Vector3 RTCPURenderer::rayColor(Ray ray, Float t0, Float t1, const std::vector<Vector2>& jittered,
                                int depth) {
    if (depth == 20) {
        return Vector3(0, 0, 0);
    }
    RayHit hit;
    const bool test = testRay(ray, t0, t1, hit);
    if (test) {
        Vector3 pixel;
        if (auto sphere = std::get_if<PlainSphere>(hit.geom)) {
            pixel = shadePlain(ray, hit, sphere->shade, jittered, depth);
        } else if (auto triangle = std::get_if<PlainTriangle>(hit.geom)) {
            pixel = shadePlain(ray, hit, triangle->shade, jittered, depth);
        } else if (auto sphere = std::get_if<Sphere>(hit.geom)) {
            Shade shade = sphere->shade;
            Vector2 uv = convertSphereTexcoord(hit.pos - sphere->center);
            uv.y() = fmod(uv.y() + 0.5, 1.0);
            Vector3 color = samplePoint(*scene.textures.get(sphere->texture), uv);
            shade.diffuse = color;
            shade.ambient = color;
            pixel = shadePlain(ray, hit, shade, jittered, depth);
        } else if (auto triangle = std::get_if<Triangle>(hit.geom)) {
            Shade shade = triangle->shade;
            Triangle3 tri3 = triangle->curve;
            Vector3 bary = triangle->curve(hit.pos);
            Vector2 uv = triangle->vA.tex * bary.x() + triangle->vB.tex * bary.y() +
                         triangle->vC.tex * bary.z();
            if (triangle->texture) {
                Vector3 color = sampleBilinear(*scene.textures.get(triangle->texture), uv);
                shade.diffuse = color;
                shade.ambient = color;
            }
            hit.normal = triangle->vA.normal * bary.x() + triangle->vB.normal * bary.y() +
                         triangle->vC.normal * bary.z();
            if (triangle->curve.sameFace(ray.dir)) {
                hit.normal *= -1;
            }

            pixel = shadePlain(ray, hit, shade, jittered, depth);
        }
        return pixel;
    } else {
        return Vector3(0, 0, 0);
    }
}

Vector3 RTCPURenderer::shadePlain(Ray ray, RayHit hit, const Shade& shade,
                                  const std::vector<Vector2>& jittered, int depth) {
    if (!nearE(shade.refractIndex, 1.0f)) {
        return shadeDielectric(ray, hit, shade, jittered, depth);
    } else {
        Vector3 pixel = scene.lightSystem.ambientIntensity * shade.ambient;
        RayHit hit2;
        const auto shadeColor = [&](const Vector3& lightV, Float intensity) {
            if (!testRay(Ray{ hit.pos, lightV, true }, 0.0001f, 1.0f / 0.0f, hit2)) {
                Vector3 h = -1 * ray.dir.normalized() + lightV;
                h.normalize();

                Float x = std::max(0.0f, lightV.dot(hit.normal));
                Float x2 = std::max(0.0f, h.dot(hit.normal));
                pixel += intensity * (x * shade.diffuse + pow(x2, shade.phong) * shade.specular);
            }
        };
        for (auto& [_, l] : scene.lightSystem.lights) {
            Vector3 lightV;
            Float intensity;
            if (auto light = std::get_if<DirectionalLight>(&l)) {
                lightV = light->v;
                intensity = light->intensity;
                shadeColor(lightV, intensity);
            } else if (auto light = std::get_if<PointLight>(&l)) {
                lightV = (light->pos - hit.pos).normalized();
                intensity = light->intensity / (light->pos - hit.pos).norm();
                shadeColor(lightV, intensity);
            } else if (auto light = std::get_if<AreaLight>(&l)) {
                for (auto& sample : jittered) {
                    Float u = sample.x() / SAMPLE_N;
                    Float v = sample.y() / SAMPLE_N;
                    Vector3 lightPos = light->pos + u * light->edge1 + v * light->edge2;
                    lightV = (lightPos - hit.pos).normalized();
                    intensity = light->intensity / (lightPos - hit.pos).norm();
                    intensity /= (SAMPLE_N * SAMPLE_N);
                    shadeColor(lightV, intensity);
                }
            }
        }

        if (fabs(shade.idealReflect.norm()) > 0.001) {
            Vector3 dir = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
            pixel += shade.idealReflect *
                     rayColor(Ray{ hit.pos, dir }, 0.0001f, 1.0f / 0.0f, jittered, depth + 1);
        }

        return pixel;
    }
}

std::vector<Vector2> RTCPURenderer::generateJittered(int n) {
    std::vector<Vector2> out;
    out.reserve(n * n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            out.push_back(Vector2(i + randUniform(), j + randUniform()));
        }
    }
    return out;
}

// 1. We find refraction ray according to Snell's law
// 2. Total internal reflection can happen, in step 1. In that case, refraction ray is not
// generated. Just relfect it.
// 3. Then we calculate the amount of light we must reflect and vice versa (refraction amount)
// according to Schlick approximation
// 4. Lastly, we simulate impurities by Beer's law. (light lose intensity as it travels)
// ------------------
// 1. Finding refraction ray
// n sin theta = n_t sin phi
// We find sin by using identity sin^2 + cos^2 = 1
// By substituting 1 - sin^2 and solving equation yields
// cos^2 phi = 1 - (n^2(1-cos^2 theta))/n_t^2
// with this in hands, we examine how to calculate the refraction ray by that qunatity
// Notice that n and b (the vector tangent to the surface) forms orthogonal basis
// and within this basis, refraction ray is (-cos phi, sin phi)
// thus, t = sin phi b - cos phi n
// we can get b by using the fact d is coplaner to the basis
// solving the equation d = B*coord
// b = (d + n cos theta)/sin theta
// Solving t using all the infoes
// t = n(d+n cos theta)/n_t - n cos phi =
// n(d-d(d.n))/n_t - n sqrt(1-n^2(1-(d.n)^2)/n_t^2)
// We assume that n = 1.0 (air)
// and n_t is the parameter (shade.refractIndex)
// 2. Total internal reflection
// Total internal reflection happens when the sqrt term is complex number
// just check if in sqrt(x) x is negative
// 3. Schlick approximation
// The method approximates Fresnel equations by following model
// R(theta) = R_0 + (1-R_0)(1-cos theta)^5
// where R_0 is ((n_t-1)/(n_t+1))^2
// this will give the amount of light we should reflect
// note that it only varies by the incident angle
// 4. Beer's law
// Beer's law is that
// dI = -CIdx
// where dx is the distance from the surface I is the light and C is parameter
// solving this differnetial equation (it's classical dy/dx = ky equation)
// I = k exp(-Cx)
// I(0) = I_0 -> I(x) = I_0exp(-Cx)
// I(1) = aI(0) -> I0a = I_0exp(-C)
// -C = lna
// I(s) = I(0)e^(ln(a)s)
// we just supply whole ln(a) as parameter (folded)
// the parameter is the shade.refractReflectance
Vector3 RTCPURenderer::shadeDielectric(Ray ray, RayHit hit, const Shade& shade,
                                       const std::vector<Vector2>& jittered, int depth) {
    hit.normal.normalize();
    ray.dir.normalize();
    Vector3 r = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;  // reflection ray
    Vector3 k;                                                         // intensity approximated
    Vector3 t;                                                         // refraction ray
    Float c;                                                           // cos theta
    if (ray.dir.dot(hit.normal) < 0) {                                 // backside
        refractRay(ray, hit.normal, shade.refractIndex, t);
        c = -ray.dir.dot(hit.normal);
        k = Vector3(1.0, 1.0, 1.0);
    } else {
        Float kx = exp(-shade.refractReflectance.x() * hit.time);
        Float ky = exp(-shade.refractReflectance.y() * hit.time);
        Float kz = exp(-shade.refractReflectance.z() * hit.time);
        k = Vector3(kx, ky, kz);
        if (refractRay(ray, -1 * hit.normal, 1 / shade.refractIndex, t)) {
            c = t.dot(hit.normal);
        } else {
            return k * rayColor(Ray{ hit.pos, r }, 0.0001f, 1.0f / 0.0f, jittered, depth + 1);
        }
    }
    Float a = (shade.refractIndex - 1);
    Float b = (shade.refractIndex + 1);
    Float R0 = (a * a) / (b * b);
    Float R = R0 + (1 - R0) * pow(1 - c, 5.0f);
    Vector3 reflectC = rayColor(Ray{ hit.pos, r }, 0.0001f, 1.0f / 0.0f, jittered, depth + 1);
    Vector3 refractC = rayColor(Ray{ hit.pos, t }, 0.0001f, 1.0f / 0.0f, jittered, depth + 1);
    return k * (R * reflectC + (1 - R) * refractC);
}

// Calculate refraction ray
bool RTCPURenderer::refractRay(Ray ray, Vector3 normal, Float index, Vector3& out) {
    Float cosTh = ray.dir.dot(normal);
    Float cosPhi2 = 1 - (1 - cosTh * cosTh) / (index * index);
    if (cosPhi2 < 0.0f) {
        return false;
    }
    Vector3 firstTerm = (ray.dir - normal * cosTh) / index;
    Vector3 secondTerm = normal * sqrt(cosPhi2);
    out = firstTerm - secondTerm;
    return true;
}

bool RTCPURenderer::testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
    const auto testGeomFunc = [&](const Geometry* geom, Ray ray, Float t0, Float t1, RayHit& hit) {
        hit.geom = geom;
        if (auto sphere = std::get_if<PlainSphere>(geom)) {
            if (ray.isShadow && !nearE(sphere->shade.refractIndex, 1.0f)) {
                return false;
            }
            return testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        } else if (auto triangle = std::get_if<PlainTriangle>(geom)) {
            return testTriangleRay(triangle->curve, ray, t0, t1, hit);
        } else if (auto sphere = std::get_if<Sphere>(geom)) {
            return testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        } else if (auto triangle = std::get_if<Triangle>(geom)) {
            return testTriangleRay(triangle->curve, ray, t0, t1, hit);
        }
    };
    return scene.geoms.testRay(ray, t0, t1, hit, testGeomFunc);
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
bool RTCPURenderer::testTriangleRay(const Triangle3& triangle, Ray ray, Float t0, Float t1,
                                    RayHit& hit) {
    Vector3 vA = triangle.pA;
    Vector3 vB = triangle.pB;
    Vector3 vC = triangle.pC;
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
