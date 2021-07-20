#include "rtcpu.h"

RTCPURenderer::RTCPURenderer(RTCPUConfig conf)
    : conf(conf), threadPool(conf.threadNum, conf.maxWidth * conf.maxHeight) {
}

RTCPURenderer::~RTCPURenderer() {
}

Scene& RTCPURenderer::sceneRef() {
    return scene;
}

void RTCPURenderer::render(Image& screen) {
    threadPool.setJobFunc([&](Vector2 pos) { renderPixel(pos, screen); });
    scene.geoms.prepare();

    for (int i = 0; i < screen.getWidth(); ++i) {
        for (int j = 0; j < screen.getHeight(); ++j) {
            Vector2 pos(i, j);
            threadPool.addJob(pos);
        }
    }
    threadPool.flush(conf.threadNum);
}

void RTCPURenderer::renderPixel(const Vector2& pos, Image& screen) {
    auto jittered = generateJittered(conf.distSampleNum);
    if (!conf.antialias) {
        const Ray ray = scene.camera.generateRay(pos, screen);
        screen.setPixel(pos, rayColor(ray, 0.0, INF, jittered));
    } else {
        Vector3 pixel;
        const size_t n = 3;
        auto jittered_ = generateJittered(n);
        for (auto& sample : jittered_) {
            Vector2 pos2 = pos;
            const Ray ray = scene.camera.generateRay(pos2, screen);
            pixel += rayColor(ray, 0.0f, INF, jittered) / (n * n);
        }
        screen.setPixel(pos, pixel);
    }
}

Vector3 RTCPURenderer::rayColor(Ray ray, Float t0, Float t1, const std::vector<Vector2>& jittered,
                                int depth) {
    RayHit hit;
    if (depth == 3) {
        return Vector3(0.0f, 0.0f, 0.0f);
    }
    if (testRay(ray, t0, t1, hit)) {
        Material material;
        if (auto sphere = hit.geom->get<PlainSphere>()) {
            material = sphere->material;
        } else if (auto triangle = hit.geom->get<PlainTriangle>()) {
            material = triangle->material;
        } else if (auto sphere = hit.geom->get<Sphere>()) {
            material = sphere->material;
            Vector2 uv = convertSphereTexcoord(hit.pos - sphere->center);
            Vector3 color = samplePoint(*scene.textures.get(sphere->texture), uv);
            material.diffuse = color;
            material.ambient = color;
        } else if (auto triangle = hit.geom->get<Triangle>()) {
            material = triangle->material;
            Vector3 bary = triangle->curve(hit.pos);
            Vector2 uv = triangle->vA.tex * bary.x() + triangle->vB.tex * bary.y() +
                         triangle->vC.tex * bary.z();
            if (triangle->texture) {
                Vector3 color = sampleBilinear(*scene.textures.get(triangle->texture), uv);
                material.diffuse = color;
                material.ambient = color;
            }
            hit.normal = triangle->vA.normal * bary.x() + triangle->vB.normal * bary.y() +
                         triangle->vC.normal * bary.z();
        }
        
        Vector3 pixel = material.diffuse*0.5;
        if (depth == 0) {
			if (material.refractIndex) {
				pixel = shadeDielectric(ray, hit, material, jittered, depth);
			} else {
				pixel = shadePhong(ray, hit, material, jittered, depth);
			}
        }
        /*if (conf.usePathtracing) {
			for (size_t i = 0; i < conf.distSampleNum * conf.distSampleNum; ++i) {
				auto sample = jittered[i];
				Vector3 dir = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
				Vector3 w = dir.normalized();
				Vector3 u = ray.dir.cross(w).normalized();
				Vector3 v = w.cross(u);
				Float j1 = sample.x()/4;
                Float j2 = sample.y() / 4;
				
				Vector3 rd = cos(2 * PI * j1) * sqrt(j2) * u + sin(2 * PI * j1) * sqrt(j2) * v +
							 sqrt(1 - j2) * w;
				Float factor = 1.0f / (conf.distSampleNum * conf.distSampleNum);
				pixel += factor *  rayColor(Ray{ hit.pos, rd }, conf.closeTime, INF, jittered, depth + 1);
			}
        }*/
		Vector3 dir = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
        //dir = hit.normal.normalized();
		Vector3 w = dir.normalized();
		Vector3 u = ray.dir.cross(w).normalized();
		Vector3 v = w.cross(u);
		Float j1 = randUniform() / 3;
		Float j2 = randUniform() / 3;
		
		Vector3 rd = cos(2 * PI * j1) * sqrt(j2) * u + sin(2 * PI * j1) * sqrt(j2) * v +
					 sqrt(1 - j2) * w;
		Float factor = 0.7f;
		pixel += factor *  rayColor(Ray{ hit.pos, rd }, conf.closeTime, INF, jittered, depth + 1);
		return pixel;
    } else {
        if (scene.environmentMap) {
            Vector2 uv = convertSphereTexcoord(ray.dir);
            return sampleBilinear(*scene.textures.get(scene.environmentMap), uv, false);
        }
        return Vector3(0.8f, 0.8f, 0.8f);
    }
}

Vector3 RTCPURenderer::shadePhong(Ray ray, RayHit hit, const Material& shade,
                                  const std::vector<Vector2>& jittered, int depth) {
	RayHit dummyHit;
    if (conf.usePathtracing) {
		  Vector3 pixel;
        Vector3 Le = shade.diffuse;
		Vector3 ko = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
        for (auto& [_, l] : scene.lightSystem.lights) {
            Vector3 ki;
            Float intensity;
            if (auto light = l.get<AreaLight>()) {
                for (auto& sample : jittered) {
                    Float u = sample.x() / conf.distSampleNum;
                    Float v = sample.y() / conf.distSampleNum;
                    Vector3 lightPos = light->pos + u * light->edge1 + v * light->edge2;
                    Vector3 lightN = light->edge1.cross(light->edge2).normalized();
                    Vector3 d = lightPos - hit.pos;
                    Float factor = 1.0f / (conf.distSampleNum * conf.distSampleNum);
                    ki = d.normalized();
                    if (!testRay(Ray{ hit.pos, d, true }, conf.closeTime, INF, dummyHit)) {
                        Float p = shade.brdf(ko, ki);
                        pixel += factor * 3 * p * Le * (std::max(hit.normal.dot(d), 0.0f)) *
                                 (-std::min(lightN.dot(d), 0.0f)) / (d.norm2() * d.norm2());
                    }
                }
            } else {
                l.unwrap(hit.pos, ki, intensity);
                // TODO
            }
        }

        if (shade.idealReflect) {
            Vector3 dir = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
            pixel += *shade.idealReflect *
                     rayColor(Ray{ hit.pos, dir }, conf.closeTime, INF, jittered, depth + 1);
        }

        return pixel;
    } else {
        Vector3 pixel = scene.lightSystem.ambientIntensity * shade.ambient;
        const auto shadeColor = [&](const Vector3& lightV, Float intensity) {
            if (!testRay(Ray{ hit.pos, lightV, true }, conf.closeTime, INF, dummyHit)) {
                Vector3 h = (-1 * ray.dir.normalized() + lightV).normalized();
                Float phongFactor = std::max(0.0f, lightV.dot(hit.normal));
                Float specFactor = std::max(0.0f, h.dot(hit.normal));
                pixel += intensity * (specFactor * shade.diffuse +
                                      pow(phongFactor, shade.phong) * shade.specular);
            }
        };
        for (auto& [_, l] : scene.lightSystem.lights) {
            Vector3 lightV;
            Float intensity;
            if (auto light = l.get<AreaLight>()) {
                for (auto& sample : jittered) {
                    Float u = sample.x() / conf.distSampleNum;
                    Float v = sample.y() / conf.distSampleNum;
                    Vector3 lightPos = light->pos + u * light->edge1 + v * light->edge2;
                    lightV = (lightPos - hit.pos).normalized();
                    intensity = light->intensity / (lightPos - hit.pos).norm();
                    intensity /= (conf.distSampleNum * conf.distSampleNum);
                    shadeColor(lightV, intensity);
                }
            } else {
                l.unwrap(hit.pos, lightV, intensity);
                shadeColor(lightV, intensity);
            }
        }

        if (shade.idealReflect) {
            Vector3 dir = ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal;
            pixel += *shade.idealReflect *
                     rayColor(Ray{ hit.pos, dir }, conf.closeTime, INF, jittered, depth + 1);
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
// and n_t is the parameter (material.refractIndex)
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
// the parameter is the material.refractReflectance
Vector3 RTCPURenderer::shadeDielectric(Ray ray, RayHit hit, const Material& shade,
                                       const std::vector<Vector2>& jittered, int depth) {
    hit.normal.normalize();
    ray.dir.normalize();
    Vector3 r =
        (ray.dir - 2 * (ray.dir.dot(hit.normal)) * hit.normal).normalized();  // reflection ray
    Vector3 k;                          // intensity approximated
    Vector3 t;                          // refraction ray
    Float c;                            // cos theta
    if (ray.dir.dot(hit.normal) < 0) {  // backside
        refractRay(ray, hit.normal, *shade.refractIndex, t);
        c = -ray.dir.dot(hit.normal);
        k = Vector3(1.0, 1.0, 1.0);
    } else {
        Float kx = exp(-shade.refractReflectance.x() * hit.time);
        Float ky = exp(-shade.refractReflectance.y() * hit.time);
        Float kz = exp(-shade.refractReflectance.z() * hit.time);
        k = Vector3(kx, ky, kz);
        if (refractRay(ray, -1 * hit.normal, 1 / *shade.refractIndex, t)) {
            c = t.dot(hit.normal);
        } else {
            return k * rayColor(Ray{ hit.pos, r }, conf.closeTime, INF, jittered, depth);
        }
    }
    Float a = (*shade.refractIndex - 1);
    Float b = (*shade.refractIndex + 1);
    Float R0 = (a * a) / (b * b);
    Float R = R0 + (1 - R0) * pow(1 - c, 5.0f);
    Vector3 reflectC = rayColor(Ray{ hit.pos, r }, conf.closeTime, INF, jittered, depth+1);
    Vector3 refractC = rayColor(Ray{ hit.pos, t }, conf.closeTime, INF, jittered, depth);
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
    out = (firstTerm - secondTerm).normalized();
    return true;
}

bool RTCPURenderer::testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
    const auto testGeomFunc = [&](const Geometry* geom, Ray ray, Float t0, Float t1, RayHit& hit) {
        hit.geom = geom;
        if (ray.isShadow && geom->material().ignoreShadow) {
            return false;
        }
        if (auto sphere = geom->get<PlainSphere>()) {
            return testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        } else if (auto triangle = geom->get<PlainTriangle>()) {
            return testTriangleRay(triangle->curve, ray, t0, t1, hit,
                                   !triangle->material.refractIndex);
        } else if (auto sphere = geom->get<Sphere>()) {
            return testSphereRay(sphere->center, sphere->radius, ray, t0, t1, hit);
        } else if (auto triangle = geom->get<Triangle>()) {
            return testTriangleRay(triangle->curve, ray, t0, t1, hit,
                                   !triangle->material.refractIndex);
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
