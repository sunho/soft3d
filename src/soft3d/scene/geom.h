#pragma once
#include <soft3d/image/texture.h>
#include <soft3d/math/curve.h>
#include <soft3d/math/linalg.h>
#include <soft3d/scene/ray.h>

#include <optional>
#include <variant>

constexpr static const Float MAX_TIME = 1000000000.0f;

struct Intersection {
    Vector3 ko;
    Vector3 diffuse;
    Vector3 sepcular;
};

static Float fresnel(Float R0, Float cosTh) {
    Float a = (1 - cosTh);
    return R0 + (1 - R0) * (a*a*a*a*a);
}

static Float fresnelDie(Float R0, Float cosTh) {
    Float a = (1 - cosTh);
    return R0 + (1 - R0) * (a*a*a*a*a);
}

static Vector3 sampleHemisphere(const Vector2& sample) {
    Float u = cos(2 * PI * sample.x()) * sqrt(sample.y());
    Float v = sin(2 * PI * sample.x()) * sqrt(sample.y());
    Float w = sqrt(1 - sample.y());
    return Vector3(u, v, w);
}

static Vector2 sampleConcentric(const Vector2& sample) {
    Vector2 u = 2.0f* sample - 1.0f;
    if (u.x() == 0.0f && u.y() == 0.0f) {
        return Vector2(0.0f, 0.0f);
    }
    Float r, th;
    if (fabs(u.x()) > fabs(u.y())) {
        r = u.x();
        th = (PI/4.0f)*(u.y()/u.x());
    } else {
        r = u.y();
        th = PI/2.0f - (PI/4.0f)*(u.x()/u.y());
    }
    return Vector2(r*cos(th), r*sin(th));
}

static Vector3 sampleHalfVector(const Vector2& sample, Float nu, Float nv, Float& phi) { 
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

struct BRDF {
    BRDF() = default;
    virtual ~BRDF() {
    }
    virtual Vector3 sample(const Intersection& intersection, Vector3& ki, Float& pdf, bool& wasSpecular) = 0;
    virtual Vector3 eval(const Intersection& intersection, const Vector3& ki) = 0;
};

struct LambertianBRDF : public BRDF {
    LambertianBRDF() = default;
    ~LambertianBRDF() {
    }

    Vector3 sample(const Intersection& intersection, Vector3& ki, Float& pdf, bool& wasSpecular) override {
        if (intersection.ko.dot(Vector3(0, 0, 1)) < 0.0f) {
            pdf = 0.0f;
            return Vector3(0, 0, 0);
        }
        ki = sampleHemisphere(Vector2(randUniform(), randUniform()));
        pdf = ki.dot(Vector3(0, 0, 1)) / PI;
        return eval(intersection, ki);
    }
    Vector3 eval(const Intersection& intersection, const Vector3& ki) override {
        if (intersection.ko.dot(Vector3(0, 0, 1)) < 0.0f) {
            return Vector3(0, 0, 0);
        }
        return intersection.diffuse * (1.0f / PI);
    }
};

struct SpecularBRDF : public BRDF {
    ~SpecularBRDF() {
    }
    Float R0;
    Vector3 sample(const Intersection& intersection, Vector3& ki, Float& pdf, bool& wasSpecular) override {
        ki = Vector3(-intersection.ko.x(), -intersection.ko.y(), intersection.ko.z());
        wasSpecular = true;
        pdf = 1.0f;
        return eval(intersection, ki);
    }
    Vector3 eval(const Intersection& intersection, const Vector3& ki) override {
        Float cosTh = intersection.ko.dot(Vector3(0,0,1));
        if (cosTh == 0.0f) {
            return Vector3(0, 0, 0);
        }
        return intersection.sepcular * fresnel(R0, cosTh) / cosTh;
    }
};

/*

 static bool refractRay(const Vector3 ko, const Vector3 normal, Float index, Vector3& ki) {
     Float cosTh = ko.dot(normal);
     Float sinTh2 = std::max(0.0f, 1-cosTh*cosTh);
     Float sinPhi2 = sinTh2 / (index * index);
     if (sinPhi2 >= 1) {
         return false;
     }
     Float cosPhi = sqrt(1-sinPhi2);
     Vector3 firstTerm = (ko - normal* cosTh) / index;
     Vector3 secondTerm = normal* cosPhi;
     ki = (firstTerm - secondTerm).normalized();
     return true;
 }

 */
static bool refractRay(const Vector3 ko, const Vector3 normal, Float index, Vector3& ki) {
    Float cosTh = ko.dot(normal);
    Float cosPhi2 = 1 - (1 - cosTh * cosTh) / (index * index);
    //printf("%f %f\n", cosPhi2, index);
    if (cosPhi2 < 0.0f) {
        // total internal reflection
        return false;
    }
    Vector3 firstTerm = (ko - normal* cosTh) / index;
    Vector3 secondTerm = normal* sqrt(cosPhi2);
    ki = (firstTerm - secondTerm).normalized();
    return true;
}

struct DielectricBRDF : public BRDF {
    DielectricBRDF(Float index) : index(index) {
        R0 = (index - 1) * (index - 1) / ((index + 1) * (index + 1));
    }
    ~DielectricBRDF() {
    }
    Float index;
    Float R0;
    Vector3 sample(const Intersection& intersection, Vector3& ki, Float& pdf, bool& wasSpecular) override {
        Vector3 normal;
        Float i;
        bool internal;
        Vector3 refract;
        Float F;
        if (intersection.ko.dot(Vector3(0, 0, 1)) > 0.0f) {
            // from outside
            i = index;
            normal = Vector3(0, 0, 1);
            internal = !refractRay(intersection.ko, normal, i, refract);
            F = fresnel(R0, intersection.ko.dot(normal));
            if (internal) {
                printf("wtf\n");
            }
        } else {
            // from inside
            i = 1.0f/index;
            normal = Vector3(0, 0, -1.0f);
            internal = !refractRay(intersection.ko, normal, i, refract);
            if (internal) {
                F = 1.0f;
            } else {
                F = fresnel(R0, -refract.dot(normal));
            }
        }
        if (randUniform() < F) {
            ki = Vector3(-intersection.ko.x(), -intersection.ko.y(), intersection.ko.z());
            wasSpecular = true;
            pdf = F;
            Float cosTh = ki.dot(normal);
            if (cosTh == 0.0f) {
                pdf = 0.0f;
                return Vector3(0,0,0);
            }
            return F * intersection.sepcular / cosTh;
        } else {
            ki = refract;
            pdf = 1 - F;
            if (ki.dot(normal) > 0) {
                printf("fuck\n");
            }
            Float cosTh = -refract.dot(normal);
            return intersection.sepcular * (1 - F) / cosTh;
        }
    }
    Vector3 eval(const Intersection& intersection, const Vector3& ki) override {
        return Vector3(0, 0, 0);
    }
};

struct CoupledBRDF : public BRDF {
    ~CoupledBRDF() {
    }
    CoupledBRDF(Float Rs, Float roughness) : Rs(Rs), roughness(roughness) {
    }
    Float Rs{ 0.1f};
    Float roughness{ 0.5f };
    Vector3 sample(const Intersection& intersection, Vector3& ki, Float& pdf, bool& wasSpecular) override {
        if (randUniform() < 0.5f) {
            ki = Vector3(-intersection.ko.x(), -intersection.ko.y(), intersection.ko.z());
            wasSpecular = true;
            pdf = 1.0f/2.0f;
        } else {
            ki = sampleHemisphere(Vector2(randUniform(), randUniform()));
            pdf = ki.dot(Vector3(0, 0, 1)) / (2*PI);
        }
        return eval(intersection, ki);
    }

    Vector3 eval(const Intersection& intersection, const Vector3& ki) override {
        Float cosTh = clamp(Vector3(0, 0, 1).dot(intersection.ko), 0.0f, 1.0f);
        Float cosThd = clamp(Vector3(0, 0, 1).dot(ki), 0.0f, 1.0f);
        if (cosTh == 0.0f) {
            return Vector3(0, 0, 0);
        }
        Vector3 specBRDF = intersection.sepcular * fresnel(Rs, cosTh) / cosTh;
        Float K = 21.0f / (20.0f * PI * (1.0f - Rs));
        Float cosThTerm = pow(1.0f - cosTh, 5.0f);
        Float cosThdTerm = pow(1.0f - cosThd, 5.0f);
        return (Rs + cosThTerm * (1.0f - Rs)) * specBRDF * Vector3(1, 1, 1) +
               K * intersection.diffuse * (1.0f - cosThTerm) * (1.0f - cosThdTerm);
    }
};

static Vector3 sampleMicrofacetHalfVector(const Vector2& sample, Float alpha, Float& phi) {
    Float l = log(1-sample[0]);
    if (isinf(l)) l = 0.0f;
    Float tan2Th = -alpha*alpha *l;
    Float cosTh = std::min(1 / sqrt(1+tan2Th), 1.0f);
    Float sinTh = sqrt(1-cosTh*cosTh);
    phi = 2*PI*sample[1];
    Float cosPhi = cos(phi);
    Float sinPhi = sin(phi);
    return Vector3(cosPhi * sinTh, sinPhi * sinTh, cosTh);
}

static Float beckmannMicrofacet(Vector3 half, Float alpha) {
    Float cosTh2 = half.z() * half.z();
    Float sinTh2 = 1 - cosTh2;
    Float tanTh2 = sinTh2 / cosTh2;
    Float tanTh = sqrt(tanTh2);
    Float cosTh = sqrt(cosTh2);
    Float d = exp(-tanTh*tanTh/(alpha*alpha));
    Float k = PI * alpha * alpha * cosTh*cosTh*cosTh*cosTh;
    return d/k;
}


struct GlossyBRDF : public BRDF {
    GlossyBRDF() = default;
    ~GlossyBRDF() {
    }
    GlossyBRDF(Float Rs, Float alpha) : Rs(Rs), alpha(alpha) {
        
    }
    Vector3 sample(const Intersection& intersection, Vector3& ki, Float& pdf, bool& wasSpecular) override {
        if (randUniform() < 0.5) {
            Float phi;
            Vector3 half = sampleMicrofacetHalfVector(Vector2(randUniform(), randUniform()), alpha, phi);
            half.normalized();
            Float koh = std::min(intersection.ko.dot(half), 1.0f);
            if (intersection.ko.z() <= 0.0f) {
                pdf = 0.0f;
                return Vector3(0,0,0);
            }
            if (koh <= 0.0f) {
                pdf = 0.0f;
                return Vector3(0,0,0);
            }
            Float cosTh = half.dot(Vector3(0,0,1));
            ki = 2 * koh * half - intersection.ko;
            pdf = 1.0f;
            pdf = beckmannMicrofacet(half, alpha) * cosTh / (4*koh);
            if (pdf <0.0f || isnan(pdf) || !isfinite(pdf)) {
                printf("asdfasdf\n");
            }
            pdf *= 0.5;
        } else {
            // Diffuse
            ki = sampleHemisphere(Vector2(randUniform(), randUniform()));
            pdf = ki.dot(Vector3(0, 0, 1)) / (PI);
            pdf *= 0.5f;
        }
        return eval(intersection, ki);
    }

    Vector3 eval(const Intersection& intersection, const Vector3& ki) override {
        Vector3 half = (intersection.ko + ki).normalized();
        Float kh = std::min(intersection.ko.dot(half), 1.0f);
        if (kh < 0.0f) {
            return Vector3(0,0,0);
        }
        if (ki.z() <= 0.0f) {
            return Vector3(0, 0, 0);
        }
        if (intersection.ko.z() <= 0.0f) {
            return Vector3(0, 0, 0);
        }
        Float D = beckmannMicrofacet(half, alpha);
        Float kin = Vector3(0,0,1).dot(ki);
        Float kon = Vector3(0, 0, 1).dot(intersection.ko);
        Float F = fresnel(Rs, kh);
        Float k = 4*kh*std::max(kin,kon);
        Vector3 specBRDF = intersection.sepcular * F * D / k;
        
        if (kin <= 0.0f) {
            return Vector3(0,0,0);
        }
        Float cosThTerm = pow(1.0f - kon / 2.0f, 5.0f);
        Float cosThdTerm = pow(1.0f - kin / 2.0f, 5.0f);
        Vector3 K = 28.0f * intersection.diffuse / (23.0f * PI);
        Vector3 diffuseBRDF = K * (1 - Rs) * (1 - cosThTerm) * (1 - cosThdTerm);
        if (specBRDF.hasNan()) {
            printf("fuck\n");
        }
        if (specBRDF.hmin() < 0.0f) {
            printf("fuck2\n");
        }
        return specBRDF + diffuseBRDF;
    }
    Float Rs;
    Float alpha;
};

struct AntPhongBRDF : public BRDF {
    AntPhongBRDF() = default;
    AntPhongBRDF(Float Rs, Float nu, Float uv) : Rs(Rs), nu(nu), nv(nv) {
    }
    ~AntPhongBRDF() {
    }
    Float Rs{};
    Float nu{};
    Float nv{};
    Vector3 sample(const Intersection& intersection, Vector3& ki, Float& pdf, bool& wasSpecular) override {
        if (randUniform() < 0.5f) {
            // Specular
            Float phi;
            Vector3 half =
                sampleHalfVector(Vector2(randUniform(), randUniform()), nu, nv, phi);
            // Convert half vector into ki
            Float koh = intersection.ko.dot(half);
            ki = 2 * koh * half - intersection.ko;
            if (ki.z() <= 0.0f) {
                pdf = 0.0f;
                return Vector3(0, 0, 0);
            }
            // Half vector pdf
            // TODO: we can just memo sqrt(nu+1) and sqrt(nv+1)
            Float k = sqrt((nu + 1) * (nv + 1)) / (2 * PI);
            Float nh = Vector3(0, 0, 1).dot(half);
            Float cosPhi = cos(phi);
            Float sinPhi = sin(phi);
            Float d = nu * cosPhi * cosPhi + nv * sinPhi * sinPhi;
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
        return eval(intersection, ki);
    }

    Vector3 eval(const Intersection& intersection, const Vector3& ki) override {
        if (ki.z() <= 0.0f) {
            return Vector3(0, 0, 0);
        }
        Vector3 half = (intersection.ko + ki).normalized();
        Float koh = intersection.ko.dot(half);
        Float kin = Vector3(0,0,1).dot(ki);
        Float kon = Vector3(0, 0, 1).dot(intersection.ko);
        Float cos2Th = half.z() * half.z();
        Float sin2Th = 1.0f - cos2Th;
        if (sin2Th == 0.0f) {
            return Vector3(0, 0, 0);
        }
        Float cos2Phi = half.x()*half.x() / sin2Th;
        Float sin2Phi = half.y()*half.y() / sin2Th;
        Float nh = Vector3(0, 0, 1).dot(half);
        Float d = (nu * cos2Phi + nv * sin2Phi);
        Float b = koh * std::max(kin, kon);
        Float k = sqrt((nu + 1) * (nv + 1)) / (8 * PI);
        Float F = fresnel(Rs, koh);
        Float dd = pow(nh, d) / b;
        Vector3 specBRDF = intersection.sepcular * k* dd * F;
        
        Float cosThTerm = pow(1.0f - kon / 2.0f, 5.0f);
        Float cosThdTerm = pow(1.0f - kin / 2.0f, 5.0f);
        Vector3 K = 28.0f * intersection.diffuse / (23.0f * PI);
        Vector3 diffuseBRDF = K * (1 - Rs) * (1 - cosThTerm) * (1 - cosThdTerm);
        return specBRDF + diffuseBRDF;
    }
};

struct PhaseFunction {
    virtual ~PhaseFunction() {
    }
    virtual Float p(const Vector3& ko, const Vector3& ki) = 0;
    virtual void samplePhase(const Vector3& ko, const Vector2& sample, Vector3& ki) = 0;
};

static Vector3 expVector(Vector3 t) {
    return Vector3(exp(t.x()), exp(t.y()), exp(t.z()));
}

struct HenyeyGreenstein : public PhaseFunction {
    explicit HenyeyGreenstein(Float g) : g(g) {
    }
    ~HenyeyGreenstein() {
    }

    Float p(const Vector3& ko, const Vector3& ki) override {
        Float cosTh = ki.dot(ko);
        Float k = 1 + g * g + 2 * g * cosTh;
        Float d = 4 * PI * k * sqrt(k);
        return (1 - g * g) / d;
    }

    void samplePhase(const Vector3& ko, const Vector2& sample, Vector3& ki) override {
        Float cosTh;
        if (g == 0.0f) {
            cosTh = 1 - 2 * sample.x();
        } else {
            Float sqrtTerm = (1 - g * g) / (1 - g+ 2 * g * sample.x());
            cosTh = (1 + g * g - sqrtTerm * sqrtTerm) / (2 * g);
        }
        //printf("%f\n",cosTh);
        Float sinTh = sqrt(std::max(0.0f, 1-cosTh*cosTh));
        Float phi = 2 * PI * sample.y();
        Vector3 del = sphericalDir(cosTh, sinTh, phi);
        ki = Basis(ko).toGlobal(del);
    }

  private:
    Float g;
};

struct Medium {
    virtual ~Medium() {
    }
    virtual Float phaseP(const Vector3& ko, const Vector3& ki) = 0;
    virtual bool sample(const Ray& ray, Float time, Vector3& weight, Ray& newRay)= 0;
    virtual Vector3 Tr(const Ray& ray, Float time) = 0;
};

struct HomoMedium : public Medium {
    HomoMedium(Vector3 sigmat, Vector3 sigmas, PhaseFunction* phase)
        : sigmat(sigmat), sigmas(sigmas), phase(phase) {
    }
    ~HomoMedium() {
    }

    Float phaseP(const Vector3& ko, const Vector3& ki) override {
        return phase->p(ko, ki);
    }

    bool sample(const Ray& ray, Float time, Vector3& weight, Ray& newRay) override {
        int channel = rand() % 3;
        Float dist = -log(1.0f - randUniform()) / sigmat[channel];
        Float t = std::min(dist, time);
        bool sampledMed = dist < time;

        Vector3 tr = expVector(-sigmat * std::min(t, MAX_TIME));
        Vector3 mpdf = sampledMed ? sigmat * tr : tr;
        Float pdf = (mpdf.x() + mpdf.y() + mpdf.z()) / 3.0f;
        if (sampledMed) {
            newRay.origin = ray.origin + ray.dir * t;
            newRay.medium = this;
            weight = sigmas * tr / pdf;
            Vector3 ki;
            phase->samplePhase(ray.dir.normalized(), Vector2(randUniform(), randUniform()), ki);
            /*if(ki.dot(ray.dir) < 0.0f) {
                printf("adfsaf\n");
            }*/
            newRay.dir = ki.normalized();
        } else {
            weight = tr / pdf;
        }
        return sampledMed;
    }

    Vector3 Tr(const Ray& ray, Float time) override {
        return expVector(-sigmat * std::min(time, MAX_TIME));
    }

    Vector3 sigmat;
    Vector3 sigmas;
    PhaseFunction* phase;
};

struct Material {
    Vector3 diffuse;
    Vector3 specular{ 1.0f, 1.0f, 1.0f };
    Float phong{ 100.0 };
    bool ignoreShadow{ false };
    BRDF* brdf{nullptr};
    Medium* medium{nullptr};
};

struct Geometry {
    Geometry() = default;
    Geometry(Material material, Image* texture) : material(material), texture(texture) {
    }
    virtual ~Geometry() {
    }
    virtual BoundingRect boundingRect() = 0;
    virtual bool rayTest(Ray ray, Float t0, Float t1, RayHit& hit) = 0;
    virtual Vector2 hitToUV(const Vector3& hit) = 0;
    virtual Triangle3* rasterizeData() = 0;
    virtual size_t rasterizeDataSize() = 0;
    Material material{};
    Image* texture{};
    bool alpha{false};
};

struct Sphere : public Geometry {
    Vector3 center;
    Float radius{ 0.0 };

    Sphere() = default;
    ~Sphere() {
    }
    explicit Sphere(Vector3 center, Float radius, Material material, Image* texture)
        : center(center), radius(radius), Geometry(material, texture) {
    }

    BoundingRect boundingRect() override {
        BoundingRect out;
        out.min = center - radius;
        out.max = center + radius;
        return out;
    }

    bool rayTest(Ray ray, Float t0, Float t1, RayHit& hit) override {
        Vector3 ec = ray.origin - center;
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
                Vector3 normal = ((hit.pos - center) / radius).normalized();
                if (ray.isShadow && normal.dot(ray.dir) > 0) {
                    return false;
                }
                hit.normal = normal;
                hit.gnormal = normal;
                hit.geom = this;
            } else {
                test = false;
            }
        }
        return test;
    }

    Vector2 hitToUV(const Vector3& hit) override {
        return Vector2(0, 0);
    }

    Triangle3* rasterizeData() override {
        return nullptr;
    }

    size_t rasterizeDataSize() override {
        return 0;
    }
};

struct TriangleVertex {
    Vector3 pos;
    Vector3 normal;
    Vector2 tex;
};

struct Triangle : public Geometry {
    TriangleVertex vA;
    TriangleVertex vB;
    TriangleVertex vC;
    Triangle3 curve;

    Triangle() = default;
    explicit Triangle(TriangleVertex a, TriangleVertex b, TriangleVertex c, Material material, Image* texture)
        : vA(a), vB(b), vC(c), curve(a.pos, b.pos, c.pos), Geometry(material, texture) {
    }

    BoundingRect boundingRect() override {
        BoundingRect out;
        out.min[0] =  std::min({ curve.pA.x(), curve.pB.x(),
                              curve.pC.x() });
        out.min[1] = std::min({ curve.pA.y(), curve.pB.y(),
                              curve.pC.y() });
        out.min[2] = std::min({ curve.pA.z(), curve.pB.z(),
                              curve.pC.z() });
        out.max[0] = std::max({ curve.pA.x(), curve.pB.x(),
                              curve.pC.x() });
        out.max[1] = std::max({ curve.pA.y(), curve.pB.y(),
                              curve.pC.y() });
        out.max[2] = std::max({ curve.pA.z(), curve.pB.z(),
                              curve.pC.z() });
        return out;
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
    bool rayTest(Ray ray, Float t0, Float t1, RayHit& hit) override {
        Vector3 pA = vA.pos;
        Vector3 pB = vB.pos;
        Vector3 pC = vC.pos;
        Vector3 ab = pB - pA;
        Vector3 ac = pC - pA;
        Vector3 normal = ab.cross(ac).normalized();
        /*if (ray.isShadow && normal.dot(ray.dir) > 0) {
            return false;
        }*/
        Float a = pA.x() - pB.x();
        Float b = pA.y() - pB.y();
        Float c = pA.z() - pB.z();
        Float d = pA.x() - pC.x();
        Float e = pA.y() - pC.y();
        Float f = pA.z() - pC.z();
        Float g = ray.dir.x();
        Float h = ray.dir.y();
        Float i = ray.dir.z();
        Float j = pA.x() - ray.origin.x();
        Float k = pA.y() - ray.origin.y();
        Float l = pA.z() - ray.origin.z();

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
        Vector3 bary = curve(hit.pos);
        hit.normal = vA.normal * bary.x() + vB.normal * bary.y() +
                     vC.normal * bary.z();
        hit.normal.normalize();
        hit.gnormal = normal;
        hit.uv = vA.tex * bary.x() + vB.tex * bary.y() +
                         vC.tex * bary.z();
        hit.geom = this;
        return true;
    }

    Vector2 hitToUV(const Vector3& hit) override {
        return Vector2(0, 0);
    }

    Triangle3* rasterizeData() override {
        return &curve;
    }

    size_t rasterizeDataSize() override {
        return 1;
    }
};

struct Light {
    Light() = default;
    virtual ~Light() {
    }
    virtual Vector3 sampleDir(const Vector3& pos) = 0;
    virtual Vector3 Le(const Vector3& brdf, const Vector3& ki, const Vector3& dir) = 0;
    virtual Vector3 Le() = 0;
};

struct DirectionalLight : public Light {
    DirectionalLight() = default;
    ~DirectionalLight() {
    }
    DirectionalLight(Vector3 intensity, Vector3 v, Float R) : intensity(intensity), v(v), R(R) {
    }
    Vector3 sampleDir(const Vector3& pos) override {
        return -10000.0f * v; // hack
    }
    Vector3 Le(const Vector3& brdf, const Vector3& ki, const Vector3& dir) override {
        Float cosTh = clamp(Vector3(0,0,1).dot(ki), 0.0f, 1.0f);
        return intensity * cosTh * clamp(-dir.normalized().dot(v), 0.0f, 1.0f) * brdf;
    }

    Vector3 Le() override {
        return intensity;
    }
    Vector3 intensity;
    Vector3 v;
    Float R;
};

struct AreaLight : public Light {
    AreaLight() = default;
    ~AreaLight() {
    }
    AreaLight(Vector3 intensity, Vector3 pos, Vector3 edge1, Vector3 edge2)
        : intensity(intensity)
        , pos(pos)
        , edge1(edge1), edge2(edge2) {
    }

    Vector3 sampleDir(const Vector3& pos) override {
        Float u = randUniform();
        Float v = randUniform();
        Vector3 lightPos = this->pos + u * edge1 + v * edge2;
        return lightPos - pos;
    }

    Vector3 Le(const Vector3& brdf, const Vector3& ki, const Vector3& dir) override {
        Float lightA = edge1.cross(edge2).norm();
        Vector3 lightN = edge1.cross(edge2).normalized();
        Float cosTh = clamp(Vector3(0,0,1).dot(ki), 0.0f, 1.0f);
        Float cosThd = clamp(-lightN.dot(dir.normalized()), 0.0f, 1.0f);
        return lightA * PI * intensity * cosTh * cosThd * brdf / dir.norm2();
    }

    Vector3 Le() override {
        return intensity;
    }

    Vector3 intensity;
    Vector3 pos;
    Vector3 edge1;
    Vector3 edge2;
};

struct DiskLight : public Light {
    DiskLight() = default;
    ~DiskLight() {
    }
    DiskLight(Vector3 intensity, Vector3 pos, Float R)
        : intensity(intensity)
        , origin(pos)
        , normal(Vector3{0,0,1}),
        R(R){
        basis = Basis(normal);
    }
    
    void lookAt(Vector3 look) {
        normal = (look - origin).normalized();
        basis = Basis(normal);
    }

    Vector3 sampleDir(const Vector3& pos) override {
        Vector2 u = R * sampleConcentric(Vector2(randUniform(), randUniform()));
        Vector3 lightPos = origin + basis.toGlobal(Vector3(u[0], u[1], 0.0f));
        return lightPos - pos;
    }

    Vector3 Le(const Vector3& brdf, const Vector3& ki, const Vector3& dir) override {
        Float lightA = R*R*PI;
        Float cosTh = clamp(Vector3(0,0,1).dot(ki), 0.0f, 1.0f);
        Float cosThd = clamp(-normal.dot(dir.normalized()), 0.0f, 1.0f);
        return lightA * intensity *cosTh*cosThd*brdf / dir.norm2();
    }

    Vector3 Le() override {
        return intensity;
    }

    Basis basis;
    Vector3 intensity;
    Vector3 origin;
    Vector3 normal;
    Float R;
};
