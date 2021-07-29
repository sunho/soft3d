#include "rtcpu.h"
#include <iostream>

RTCPURenderer::RTCPURenderer(RTCPUConfig conf)
    : conf(conf), threadPool(conf.threadNum, conf.maxWidth * conf.maxHeight) {
}

RTCPURenderer::~RTCPURenderer() {
}

Scene& RTCPURenderer::sceneRef() {
    return scene;
}

Float fresnel(Float R0, Float cosTh) {
    Float a = (1 - cosTh);
    return R0 + (1 - R0) * (a*a*a*a*a);
}

void RTCPURenderer::render(Image& screen) {
    threadPool.setJobFunc([&](Vector2 pos) { renderPixel(pos, screen); });
    scene.geoms.prepare();
    std::atomic<bool> quit{ false };
    std::thread timerThread([&]() { 
        using namespace std::chrono;
        int prevRays = getProcessedRays();
        auto next = std::chrono::system_clock::now() + 1s;
        while (!quit.load(std::memory_order_relaxed)) {
            int nextRays = getProcessedRays();
            printf("Rays per seconds: %d\n", nextRays - prevRays);
            prevRays = nextRays;
            std::this_thread::sleep_until(next);
            next += 1s;
        }
    });
    for (int i = 0; i < screen.getWidth(); ++i) {
        for (int j = 0; j < screen.getHeight(); ++j) {
            Vector2 pos(i, j);
            threadPool.addJob(pos);
        }
    }
    threadPool.flush(conf.threadNum);
    quit.store(true, std::memory_order_acquire);
    timerThread.join();
    screen.sigmoidToneMap(0.05f);
}

void RTCPURenderer::renderPixel(const Vector2& pos, Image& screen) {
    const Ray ray = scene.camera.generateRay(pos, screen);
    Vector3 pixel;
    for (int i = 0; i < conf.pathSampleNum; ++i) {
        pixel += rayColor(ray, Vector2(0.0f,0.0f));
    }
    pixel /= conf.pathSampleNum;
    screen.setPixel(pos, pixel);
}

Vector3 RTCPURenderer::rayColor(Ray ray, Vector2 sample) {
    // L_s(ko) = brdf(kj,ko)L_s(kj)cos theta_j/pdf(k_j)
    // For the last vertex we convert this form into surface area form so that we can sample area
    // light L_s(last) = L_e(x')cos theta cos theta_j
    int bounce = 0;
    Vector3 weight(1.0f,1.0f,1.0f);
    Float t0 = 0.0f;
    Float t1 = INF;
    Vector3 pixel;

    const auto getTBNBasis = [&](Vector3 normal, Vector3 dir) {
        Vector3 w = normal.normalized();
        Vector3 u = w.x() > w.y() ? Vector3(0, 1, 0).cross(w).normalized() : Vector3(1, 0, 0).cross(w).normalized();
        Vector3 v = w.cross(u);
        return Basis{ u, v, w };
    };
    RayHit hit;
    Material material;
    Basis TBN;
    bool wasSpecular=false;
    Vector3 ko;
    while (true) {
        if (!testRayAndFetchTex(ray, t0, t1, hit, material)) {
            Vector2 uv = convertSphereTexcoord(ray.dir);
            Vector3 Le = sampleBilinear(*scene.textures.get(scene.environmentMap), uv, false);
            pixel += weight * Le;
            break;
        }
        if (wasSpecular) {
            Vector3 Le = sampleLight(material, hit.pos, hit.normal, ko, TBN); 
            pixel += weight * Le;
        }
        Vector3 invDir = -1*ray.dir;
        TBN = getTBNBasis(hit.normal, invDir);
        // normal coord = Basis^T * worldCoord
        // = (u . dir, v . dir, w. dir)
        ko = Vector3(TBN.u.dot(invDir), TBN.v.dot(invDir), TBN.w.dot(invDir));
        Vector3 ki;
        Float pdf;
        Vector3 brdf = sampleBRDF(material, ko, ki, pdf, wasSpecular);
        t0 = conf.closeTime;
        ray.origin = hit.pos;
        auto nextDir = ki.x() * TBN.u + ki.y() * TBN.v + ki.z() * TBN.w;
        ray.dir = nextDir;
        if (pdf == 0.0f) {
            break;
        }
        
        if (!wasSpecular) {
            Vector3 Le = sampleLight(material, hit.pos, hit.normal, ko, TBN); 
            pixel += weight * Le;
        }
        weight *= brdf * hit.normal.dot(invDir) / pdf;

        ++bounce;
        if (bounce == 3) {
            break;
        }
    }

    return pixel;
}

Vector3 RTCPURenderer::sampleLight(const Material& material, const Vector3& pos, const Vector3& normal, const Vector3& ko, const Basis& TBN) {
    RayHit hit;
    auto l = scene.lightSystem.lights.draw();
    if (auto light = l->get<AreaLight>()) {
        Float u = randUniform();
        Float v = randUniform();

        Vector3 lightPos = light->pos + u * light->edge1 + v * light->edge2;
        Vector3 lightN = light->edge1.cross(light->edge2).normalized();
        Vector3 d = lightPos - pos;
        Float intensity = (light->intensity) / d.norm2();
        Vector3 dd = d.normalized();
        Vector3 ki = Vector3(TBN.u.dot(dd), TBN.v.dot(dd), TBN.w.dot(dd));
        if (!testRay(Ray{ pos, d, false }, conf.closeTime, 1.0f - conf.closeEpsillon, hit)) {
            Vector3 Le = Vector3(0xfff9c9) * intensity;
            Float cosTh = clamp(normal.dot(dd), 0.0f, 1.0f);
            Float cosThd = clamp(-lightN.dot(dd), 0.0f, 1.0f);
            Vector3 brdf = evalBRDF(material, ko, ki);
            return Le * cosTh * cosThd * brdf / d.norm2();
        }
    }
    return Vector3(0.0f, 0.0f, 0.0f);
}

Vector3 RTCPURenderer::sampleBRDF(const Material& material, const Vector3& ko, Vector3& ki, Float& pdf, bool& specular) { 
    specular = false;
    if (auto brdf = std::get_if<LambertianBRDF>(&material.brdf)) {
        ki = sampleHemisphere(Vector2(randUniform(), randUniform()));
        pdf = ki.dot(Vector3(0, 0, 1)) / PI;
    } else if (auto brdf = std::get_if<SpecularBRDF>(&material.brdf)) {
        ki = Vector3(-ko.x(), -ko.y(), ko.z());
        specular = true;
        pdf = 1.0f;
    } else if (auto brdf = std::get_if<CoupledBRDF>(&material.brdf)) {
        if (randUniform() < 0.5f) {
            ki = Vector3(-ko.x(), -ko.y(), ko.z());
            specular = true;
            pdf = 1.0f/2.0f;
        } else {
            ki = sampleHemisphere(Vector2(randUniform(), randUniform()));
            pdf = ki.dot(Vector3(0, 0, 1)) / (2*PI);
        }
    } else if (auto brdf = std::get_if<AntPhongBRDF>(&material.brdf)) {
        if (randUniform() < 0.5f) {
            // Specular
            Float phi;
            Vector3 half =
                sampleHalfVector(Vector2(randUniform(), randUniform()), brdf->nu, brdf->nv, phi);
            // Convert half vector into ki
            Float koh = ko.dot(half);
            ki = 2 * koh * half - ko;
            if (ki.z() <= 0.0f) {
                pdf = 0.0f;
                return Vector3(0, 0, 0);
            }
            // Half vector pdf
            // TODO: we can just memo sqrt(nu+1) and sqrt(nv+1)
            Float k = sqrt((brdf->nu + 1) * (brdf->nv + 1)) / (2 * PI);
            Float nh = Vector3(0, 0, 1).dot(half);
            Float cosPhi = cos(phi);
            Float sinPhi = sin(phi);
            Float d = brdf->nu * cosPhi * cosPhi + brdf->nv * sinPhi * sinPhi;
            // PDF should be multiplied by 1 / |J(ko, h)|
            // J(ko,h) = 4 * ko.h
            pdf = k * pow(nh, d) / (4 * koh);
            pdf *= 0.5f;

        } else {
            // Diffuse
            ki = sampleHemisphere(Vector2(randUniform(), randUniform()));
            pdf = ki.dot(Vector3(0, 0, 1)) / (PI);
            pdf *= 0.5f;
        }
    }
    return evalBRDF(material, ko, ki);
}

Vector3 RTCPURenderer::evalBRDF(const Material& material, const Vector3& ko, const Vector3& ki) {
    if (auto brdf = std::get_if<LambertianBRDF>(&material.brdf)) {
        return material.diffuse * (1.0f / PI);
    } else if (auto brdf = std::get_if<SpecularBRDF>(&material.brdf)) {
        Float cosTh = ko.dot(Vector3(0,0,1));
        if (cosTh == 0.0f) {
            return Vector3(0, 0, 0);
        }
        return Vector3(1,1,1) * fresnel(brdf->R0, cosTh) / cosTh;
    } else if (auto brdf = std::get_if<CoupledBRDF>(&material.brdf)) {
        Vector3 half = (ko +  ki).normalized();
        Float cosTh = clamp(Vector3(0,0,1).dot(ko), 0.0f, 1.0f);
        Float cosThd = clamp(Vector3(0,0,1).dot(ki), 0.0f, 1.0f);
        Float nh = clamp(Vector3(0,0,1).dot(half),0.0f,1.0f);
        Float a = nh * brdf->roughness;
        Float k = brdf->roughness / (1.0f - nh * nh + a * a);
        Float specBRDF = k * k * (1.0f / PI); // ndf
        Float R0 = brdf->R0;
        Float K = 21.0f / (20.0f * PI * (1.0f - R0));
        Float cosThTerm = pow(1.0f - cosTh, 5.0f);
        Float cosThdTerm = pow(1.0f - cosThd, 5.0f);
        return (R0 + cosThTerm * (1.0f - R0)) * specBRDF* Vector3(1,1,1) +
            K* material.diffuse*(1.0f - cosThTerm) * (1.0f - cosThdTerm);
    } else if (auto brdf = std::get_if<AntPhongBRDF>(&material.brdf)) {
        if (ki.z() <= 0.0f) {
            return Vector3(0, 0, 0);
        }
        Vector3 half = (ko + ki).normalized();
        Float koh = ko.dot(half);
        Float kin = Vector3(0,0,1).dot(ki);
        Float kon = Vector3(0,0,1).dot(ko);
        Float cos2Th = half.z() * half.z();
        Float sin2Th = 1.0f - cos2Th;
        if (sin2Th == 0.0f) {
            return Vector3(0, 0, 0);
        }
        Float cos2Phi = half.x()*half.x() / sin2Th;
        Float sin2Phi = half.y()*half.y() / sin2Th;
        Float nh = Vector3(0, 0, 1).dot(half);
        Float d = (brdf->nu * cos2Phi + brdf->nv * sin2Phi);
        Float b = koh * std::max(kin, kon);
        Float k = sqrt((brdf->nu + 1) * (brdf->nv + 1)) / (8 * PI);
        Float F = fresnel(brdf->Rs, 1.0f-koh);
        Float dd = pow(nh, d) / b;
        Vector3 specBRDF = Vector3(1, 1, 1) * k* dd * F;
        
        Float cosThTerm = pow(1.0f - kon / 2.0f, 5.0f);
        Float cosThdTerm = pow(1.0f - kin / 2.0f, 5.0f);
        Vector3 K = 28.0f * material.diffuse / (23.0f * PI);
        Vector3 diffuseBRDF = K * (1 - brdf->Rs) * (1 - cosThTerm) * (1 - cosThdTerm);
        return specBRDF + diffuseBRDF;
     }
    return material.diffuse * (1.0f / PI);
}
Vector3 RTCPURenderer::sampleHemisphere(const Vector2& sample) {
    Float u = cos(2 * PI * sample.x()) * sqrt(sample.y());
    Float v = sin(2 * PI * sample.x()) * sqrt(sample.y());
    Float w = sqrt(1 - sample.y());
    return Vector3(u, v, w);
}

// we generate half vector according to pdf of ant phong model for 0 < phi < pi/2
// and map to each quadrant by sample.x 
Vector3 RTCPURenderer::sampleHalfVector(const Vector2& sample, Float nu, Float nv, Float& phi) { 
    Float delPhi;
    Float j1;
    Float j2 = sample.y();
    if (sample.x() < 0.25f) {
        delPhi = 0.0f;
        j1 = 4.0f * sample.x();
    } else if (sample.x() < 0.5f) {
        delPhi = PI/2.0f;
        j1 = 4.0f * sample.x() - 1.0f;
    } else if (sample.x() < 0.75f) {
        delPhi = PI;
        j1 = 4.0f * sample.x() - 2.0f;
    } else {
        delPhi = 3.0f*PI/2.0f;
        j1 = 4.0f * sample.x() - 3.0f;
    }
    Float k = sqrt((nu + 1) / (nv + 1));
    phi = atan(k * tan(PI * j1 / 2.0f)) + delPhi;
    Float cosPhi = cos(phi);
    Float sinPhi = sin(phi);
    // adding k*(PI/2) doesn't change the ratio of cos th and sin th
    // so it's safe to just use offseted phi
    Float cosTh = pow((1-j2),1.0f/(nu*cosPhi*cosPhi+nv*sinPhi*sinPhi+1));
    // th in 0~pi/2
    Float th = acos(cosTh);
    // Other ways to get sinTh from cosTh requries sqrt with squares
    // TODO: might exist faster way
    Float sinTh = sin(th);
    return Vector3(cosPhi * sinTh, sinPhi * sinTh, cosTh);
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

// Calculate refraction ray
bool RTCPURenderer::refractRay(Ray ray, Vector3 normal, Float index, Vector3& out) {
    Float cosTh = ray.dir.dot(normal);
    Float cosPhi2 = 1 - (1 - cosTh * cosTh) / (index * index);
    if (cosPhi2 < 0.0f) {
        return false;
    }
    Vector3 firstTerm = (ray.dir - normal * cosTh) / index;
    Vector3 secondTerm = normal * sqrt(cosPhi2);
    out = (firstTerm - secondTerm).normalized();
    return true;
}

bool RTCPURenderer::testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
    ++processedRays;
    const auto testGeomFunc = [&](const Geometry* geom, Ray ray, Float t0, Float t1, RayHit& hit) {
        hit.geom = geom;
        if (ray.isShadow && geom->material().ignoreShadow) {
            return false;
        }
        if (auto sphere = geom->get<PlainSphere>()) {
            return testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        } else if (auto triangle = geom->get<PlainTriangle>()) {
            return testTriangleRay(triangle->curve, ray, t0, t1, hit,
                                   true);
        } else if (auto sphere = geom->get<Sphere>()) {
            return testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        } else if (auto triangle = geom->get<Triangle>()) {
            return testTriangleRay(triangle->curve, ray, t0, t1, hit,
                                   true);
        }
    };
    return scene.geoms.testRay(ray, t0, t1, hit, testGeomFunc);
}

bool RTCPURenderer::testRayAndFetchTex(Ray ray, Float t0, Float t1, RayHit& hit,
                                       Material& material) {
    if (testRay(ray, t0, t1, hit)) {
        if (auto sphere = hit.geom->get<PlainSphere>()) {
            material = sphere->material;
        } else if (auto triangle = hit.geom->get<PlainTriangle>()) {
            material = triangle->material;
        } else if (auto sphere = hit.geom->get<Sphere>()) {
            material = sphere->material;
            Vector2 uv = convertSphereTexcoord(hit.pos - sphere->center);
            Vector3 color = samplePoint(*scene.textures.get(sphere->texture), uv);
            material.diffuse = color;
        } else if (auto triangle = hit.geom->get<Triangle>()) {
            material = triangle->material;
            Vector3 bary = triangle->curve(hit.pos);
            Vector2 uv = triangle->vA.tex * bary.x() + triangle->vB.tex * bary.y() +
                         triangle->vC.tex * bary.z();
            if (triangle->texture) {
                Vector3 color = sampleBilinear(*scene.textures.get(triangle->texture), uv);
                material.diffuse = color;
            }
            hit.normal = triangle->vA.normal * bary.x() + triangle->vB.normal * bary.y() +
                         triangle->vC.normal * bary.z();
        }
        return true;
    } else {
        return false;
    }
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
                                    RayHit& hit, bool singleSide) {
    Vector3 vA = triangle.pA;
    Vector3 vB = triangle.pB;
    Vector3 vC = triangle.pC;
    Vector3 ab = vB - vA;
    Vector3 ac = vC - vA;
    Vector3 normal = ab.cross(ac).normalized();
    if (singleSide && normal.dot(ray.dir) > 0) {
        return false;
    }
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
    hit.normal = normal;
    return true;
}
