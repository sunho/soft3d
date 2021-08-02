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
    Intersection ins;
    RayHit hit;
    Basis TBN;
    bool wasSpecular=false;
    while (true) {
        if (!testRay(ray, t0, t1, hit)) {
            Vector2 uv = convertSphereTexcoord(ray.dir.normalized());
            Vector3 Le = 2.18*PI*sampleBilinear(*scene.environmentMap, uv, true);
            pixel += weight*Le;
            if (wasSpecular) {
                for (auto light : scene.lights.list()) {
                    Vector3 dir = light->sampleDir(hit.pos);
                    RayHit dummy;
                    if (!testRay(Ray{ hit.pos, dir, false }, conf.closeTime, 1.0f - conf.closeEpsillon, dummy)) {
                        Vector3 dd = dir.normalized();
                        Vector3 ki = Vector3(TBN.u.dot(dd), TBN.v.dot(dd), TBN.w.dot(dd));
                        Vector3 brdf = hit.geom->material.brdf->eval(ins, ki);
                        pixel += weight*light->Le(brdf, ki, dir);
                    }
                }
            }
            break;
        }
        Vector3 invDir = -1*ray.dir;
        TBN = getTBNBasis(hit.normal, invDir);
        ins.sepcular = hit.geom->material.specular;
        ins.diffuse = hit.geom->material.diffuse;
        if (hit.geom->texture) {
            ins.diffuse = sampleBilinear(*hit.geom->texture, hit.uv);
        }
        ins.ko = Vector3(TBN.u.dot(invDir), TBN.v.dot(invDir), TBN.w.dot(invDir));
        if (wasSpecular) {
            Vector3 Le = sampleLight(hit.geom, ins,hit.pos, hit.normal, ins.ko, TBN); 
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
        if (pdf == 0.0f) {
            break;
        }
        if (hit.normal.dot(invDir) <= 0.0f) {
            break;
        }
        
        if (!wasSpecular) {
            Vector3 Le = sampleLight(hit.geom, ins, hit.pos, hit.normal, ins.ko, TBN); 
            pixel += weight * Le;
        }
        weight *= brdf * hit.normal.dot(ray.dir) / pdf;

        ++bounce;
        if (bounce == 4) {
            break;
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
    if (!testRay(Ray{ pos, dir, false }, conf.closeTime, 1.0f - conf.closeEpsillon, hit)) {
        Vector3 dd = dir.normalized();
        Vector3 ki = Vector3(TBN.u.dot(dd), TBN.v.dot(dd), TBN.w.dot(dd));
        Vector3 brdf = geom->material.brdf->eval(ins, ki);
        return light->Le(brdf, ki, dir);
    }
    return Vector3(0, 0, 0);
}
    
bool RTCPURenderer::testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
    bool out = scene.bvhtree.testRay(ray, t0, t1, hit);
    ++processedRays;
    return out;
}
