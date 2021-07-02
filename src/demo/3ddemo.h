#include <GLFW/glfw3.h>
#include <focg/backend/rtcpu/renderer.h>
#include <focg/backend/zcpu/renderer.h>
#include <focg/common/curve.h>
#include <focg/common/image.h>
#include <focg/common/scene.h>
#include <focg/common/transform.h>
#include <focg/engine/engine.h>
#include <focg/engine/loader.h>
#include <glad/glad.h>

#include <chrono>
#include <thread>

static EngineConfig engineConf = { .width = 500,
                                   .height = 500,
                                   .fps = 60,
                                   .renderer = Backend::ZCPU };

struct ObjLoad : public App {
    Model model;

    ObjLoad() {
    }
    ~ObjLoad() {
    }

    void init(Engine& engine, Scene& scene) override {
        model = loadObj("model.obj");
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.0, 1.5), basis, 1.0);
        StdLightSystem lightSystem;
        lightSystem.ambientIntensity = 0.1;
        lightSystem.lights.push_back({ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() });
        lightSystem.lights.push_back({ 0.3, Vector3(-0.8, -1.0, 0.5).normalized() });
        scene.lightSystem = lightSystem;

        Shade shade1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
                         .ambient = Vector3(0.5, 1.0, 0.5),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .reflect = Vector3(0.2, 0.2, 0.2),
                         .phong = 100.0 };

        Shade shade2 = { .diffuse = Vector3(0.5, 0.5, 1.0),
                         .ambient = Vector3(0.5, 0.5, 1.0),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .reflect = Vector3(0.2, 0.2, 0.2),
                         .phong = 100.0 };
        Shade shade3 = { .diffuse = Vector3(0.7, 0.7, 0.7),
                         .ambient = Vector3(0.7, 0.7, 0.7),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .reflect = Vector3(0.2, 0.2, 0.2),
                         .phong = 100.0 };

        Float sz = -0.3;
        scene.geoms.push_back(Triangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0, sz),
                                       Vector3(2.0, 2.0, sz), shade3));
        scene.geoms.push_back(Triangle(Vector3(-1.0, -1.0, sz), Vector3(1.0, -2.0, sz),
                                       Vector3(2.0, 2.0, sz), shade3));
        scene.geoms.push_back(Sphere(Vector3(0.0, 0.0, -0.15), 0.10, shade2));

        for (auto tri : model.meshes[0].data) {
            scene.geoms.push_back(Triangle(tri.a, tri.b, tri.c, shade1));
        }
    }

    void update(Engine& engine, Scene& scene, double dt) override {
    }
};

struct SphereRayTrace : public App {
    SphereRayTrace() {
    }
    ~SphereRayTrace() {
    }

    void init(Engine& engine, Scene& scene) override {
        Shade shade1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
                         .ambient = Vector3(0.5, 1.0, 0.5),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .phong = 100.0 };
        Shade shade2 = { .diffuse = Vector3(0.5, 0.5, 1.0),
                         .ambient = Vector3(0.5, 0.5, 1.0),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .phong = 100.0 };
        Shade shade3 = { .diffuse = Vector3(0.5, 0.5, 0.5),
                         .ambient = Vector3(0.5, 0.5, 0.5),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .reflect = Vector3(0.2, 0.2, 0.2),
                         .phong = 100.0 };
        Shade shade4 = { .diffuse = Vector3(0.8, 0.8, 0.8),
                         .ambient = Vector3(0.8, 0.8, 0.8),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .reflect = Vector3(0.2, 0.2, 0.2),
                         .phong = 100.0 };
        StdLightSystem lightSystem;
        lightSystem.ambientIntensity = 0.2;
        lightSystem.lights.push_back({ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() });
        lightSystem.lights.push_back({ 0.3, Vector3(-0.8, -1.0, 0.5).normalized() });
        lightSystem.lights.push_back({ 0.3, Vector3(0.8, -1.0, 0.5).normalized() });
        scene.lightSystem = lightSystem;

        scene.geoms.push_back(Sphere(Vector3(-0.2, 0.0, 0.0), 0.25, shade1));
        scene.geoms.push_back(Sphere(Vector3(0.3, 0.0, -0.4), 0.25, shade2));
        scene.geoms.push_back(Triangle(Vector3(-0.5, 1.0, -0.2), Vector3(-0.5, -0.5, -0.1),
                                       Vector3(0.5, 0.5, -0.5), shade4));
        Float sz = -0.6;
        scene.geoms.push_back(Triangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0, sz),
                                       Vector3(2.0, 2.0, sz), shade3));
        scene.geoms.push_back(Triangle(Vector3(-1.0, -1.0, sz), Vector3(1.0, -2.0, sz),
                                       Vector3(2.0, 2.0, sz), shade3));
        Sphere& sp = std::get<Sphere>(scene.geoms[0]);
        sp.transform = toHomo(scale3(1.0, 0.5, 1.0));
        sp.itransform = *invertMatrix4x4(sp.transform);

        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(1.5, 1.5, 2.0), basis, 1.0);
    }

    void update(Engine& engine, Scene& scene, double dt) override {
        Camera& camera = scene.camera;
        if (engine.pressed(GLFW_KEY_W)) {
            camera.e.z() -= 0.05;
        }
        if (engine.pressed(GLFW_KEY_S)) {
            camera.e.z() += 0.05;
        }
        if (engine.pressed(GLFW_KEY_A)) {
            camera.e.x() -= 0.05;
        }
        if (engine.pressed(GLFW_KEY_D)) {
            camera.e.x() += 0.05;
        }
    }
};

#define RUN3D(demo)                                                                                \
    {                                                                                              \
        Engine engine(engineConf);                                                                 \
        App* d = new demo;                                                                         \
        engine.run(d);                                                                             \
    }
