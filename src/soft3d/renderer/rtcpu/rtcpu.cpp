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

void RTCPURenderer::render(Image& screen) {
    threadPool.setJobFunc([&](Vector2 pos) { renderPixel(pos, screen); });
    scene.prepare();
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
    screen.sigmoidToneMap(0.1f);
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

    Intersection ins;
    RayHit hit;
    Basis TBN;
    bool wasSpecular=false;
    while (true) {

        ++bounce;
        if (bounce == 10) {
            break;
        }
        if (!testRay(ray, t0, t1, hit)) {
            Vector2 uv = convertSphereTexcoord(ray.dir.normalized());
            Vector3 Le = 4*PI*sampleBilinear(*scene.environmentMap, uv, true);
            pixel += weight*Le;
            break;
        }
        Vector3 invDir = -1*ray.dir;
        if (hit.gnormal.dot(invDir) <= 0.0f) {
            ray.medium = hit.geom->material.medium;
        }
        TBN = Basis(hit.gnormal.normalized());
        if (ray.medium) {
            Vector3 w;
            Ray newRay;
            bool wasMed = ray.medium->sample(ray, hit.time, w, newRay);
            weight *= w;
            if (weight.isZero()) {
                break;
            }
            if (weight.hmin() < 0.0f) {
                break;
            }
            if (wasMed) {
                ray = newRay;
                Vector3 Le = sampleMediumLight(ray, ray.medium, ray.origin); 
                pixel += weight * Le;
                continue;
            } else {
                ray.medium = nullptr;
            }
        }
        if (hit.geom->material.brdf) {
            ins.sepcular = hit.geom->material.specular;
            ins.diffuse = hit.geom->material.diffuse;
            if (hit.geom->texture) {
                ins.diffuse = sampleBilinear(*hit.geom->texture, hit.uv);
            }
            ins.ko = Vector3(TBN.u.dot(invDir), TBN.v.dot(invDir), TBN.w.dot(invDir));
            if (wasSpecular) {
                Vector3 Le = sampleLight(hit.geom, ins, hit.pos, hit.normal, ins.ko, TBN);
                pixel += weight * Le;
            }
            // normal coord = Basis^T * worldCoord
            // = (u . dir, v . dir, w. dir)
            Vector3 ki;
            Float pdf;
            Vector3 brdf = hit.geom->material.brdf->sample(ins, ki, pdf, wasSpecular);
            t0 = conf.closeTime;
            ray.origin = hit.pos;
            Vector3 nextDir = ki.x() * TBN.u + ki.y() * TBN.v + ki.z() * TBN.w;
            ray.dir = nextDir;
            ray.dir.normalize();
            if (pdf == 0.0f) {
                break;
            }

            if (!wasSpecular) {
                Vector3 Le = sampleLight(hit.geom, ins, hit.pos, hit.normal, ins.ko, TBN);
                pixel += weight * Le;
            }
            weight *= brdf * fabs(hit.normal.dot(ray.dir)) / pdf;
            if (weight.isZero()) {
                break;
            }
            if (weight.hmin() < 0.0f) {
                break;
            }
            //ray.origin += ray.dir * conf.closeTime;
        } else {
            ray.origin += ray.dir * (hit.time + conf.closeTime);
        }
    }

    return pixel;
}

Vector3 RTCPURenderer::sampleLight(Geometry* geom, const Intersection& ins, const Vector3& pos, const Vector3& normal,
                                   const Vector3& ko, const Basis& TBN) {
    auto& lights = scene.lights.list();
    size_t i = rand() % lights.size();
    Light* light = lights[i];
    Vector3 dir = light->sampleDir(pos);
    RayHit hit;
    if (!testRay(Ray{ pos, dir, true }, conf.closeTime, 1.0f - conf.closeEpsillon, hit)) {
        Vector3 dd = dir.normalized();
        Vector3 ki = Vector3(TBN.u.dot(dd), TBN.v.dot(dd), TBN.w.dot(dd));
        Vector3 brdf = geom->material.brdf->eval(ins, ki);
        return light->Le(brdf, ki, dir);
    }
    return Vector3(0, 0, 0);
}
    
Vector3 RTCPURenderer::sampleMediumLight(const Ray& ray, Medium* medium, const Vector3& pos) {
    auto& lights = scene.lights.list();
    size_t i = rand() % lights.size();
    Light* light = lights[i];
    Vector3 dir = light->sampleDir(pos);
    RayHit hit;
    //if (!testRay(Ray{ pos, dir, true }, conf.closeTime, 1.0f - conf.closeEpsillon, hit)) {
        Ray ray2{ pos, dir };
        if (testRay(ray2, 0.0f, INF, hit)) {
            Vector3 dd = dir.normalized();
            Vector3 tr = medium->Tr(ray2, hit.time);
            return light->Le() * tr / medium->phaseP(ray.dir.normalized(), dd);
        }
        //printf("asdfsaf\n");
    //}
    return Vector3(0, 0, 0);
}
    
bool RTCPURenderer::testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
    bool out = scene.bvhtree.testRay(ray, t0, t1, hit);
    ++processedRays;
    return out;
}
