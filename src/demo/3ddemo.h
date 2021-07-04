#include <nuguri3d/backend/rtcpu/renderer.h>
#include <nuguri3d/backend/zcpu/renderer.h>
#include <nuguri3d/common/curve.h>
#include <nuguri3d/common/image.h>
#include <nuguri3d/common/scene.h>
#include <nuguri3d/common/transform.h>
#include <nuguri3d/engine/engine.h>
#include <nuguri3d/engine/loader.h>

#include <chrono>
#include <thread>

static EngineConfig engineConf = { .width = 500,
                                   .height = 500,
                                   .fps = 60,
                                   /*  .aaProfile = AAProfile {
                                         .filter = boxFilter1(1),
                                     },*/
                                   .renderer = Backend::ZCPU };

struct ObjLoad : public App {
    Model model;

    ObjLoad() {
    }
    ~ObjLoad() {
    }

    void init(Engine& engine, Scene& scene) override {
        Image base = loadTexture("base.png");
        TextureId baseId = scene.textures.move(std::move(base));
        model = loadObj("model.obj");
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.0, 1.5), basis, 1.0);
        LightSystem lightSystem{};
        lightSystem.ambientIntensity = 0.1;
        lightSystem.lights.move(
            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));
        lightSystem.lights.move(
            std::move(DirectionalLight{ 0.3, Vector3(-0.8, -1.0, 0.5).normalized() }));
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
        Shade shadewall = { .diffuse = Vector3(0.7, 0.7, 0.7),
                            .ambient = Vector3(0.7, 0.7, 0.7),
                            .specular = Vector3(0.3, 0.3, 0.3),
                            .reflect = Vector3(0.2, 0.2, 0.2),
                            .phong = 100.0 };

        Float sz = -0.3;
        scene.geoms.push_back(PlainTriangle(Vector3(-2.0, 0.3, -1.0), Vector3(2.0, -1.0, 1.0),
                                            Vector3(2.0, 0.3, -1.0), shadewall));
        scene.geoms.push_back(PlainTriangle(Vector3(-2.0, 0.3, -1.0), Vector3(-2.0, -1.0, 1.0),
                                            Vector3(2.0, -1.0, 1.0), shadewall));
        scene.geoms.push_back(PlainSphere(Vector3(0.0, 0.0, -0.15), 0.10, shade2));

        for (auto tri : model.meshes[0].data) {
            auto tt = Triangle({ tri.a, tri.nA, tri.tA }, { tri.b, tri.nB, tri.tB },
                               { tri.c, tri.nC, tri.tC }, shade1);
            tt.texture = baseId;
            scene.geoms.push_back(tt);
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
        Shade shade1 = { .diffuse = Vector3(0x9dcdfc),
                         .ambient = Vector3(0x9dcdfc),
                         .specular = Vector3(0.1, 0.1, 0.1),
                         .phong = 100.0 };
        Shade shade2 = { .diffuse = Vector3(0xfb9dfc),
                         .ambient = Vector3(0xfb9dfc),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .phong = 100.0 };
        Shade shadewall = { .diffuse = Vector3(0.3, 0.3, 0.3),
                            .ambient = Vector3(0.8, 0.8, 0.8),
                            .specular = Vector3(0.3, 0.3, 0.3),
                            .reflect = Vector3(0.2, 0.2, 0.2),
                            .phong = 100.0 };
        Shade shadetri = { .diffuse = Vector3(0xd3e4f5),
                           .ambient = Vector3(0xd3e4f5),
                           .specular = Vector3(0.3, 0.3, 0.3),
                           .reflect = Vector3(0.2, 0.2, 0.2),
                           .phong = 100.0 };
        LightSystem lightSystem;
        lightSystem.ambientIntensity = 0.2;
        lightSystem.lights.move(
            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));
        lightSystem.lights.move(
            std::move(DirectionalLight{ 0.3, Vector3(-0.8, -1.0, 0.5).normalized() }));
        lightSystem.lights.move(
            std::move(DirectionalLight{ 0.3, Vector3(0.8, -1.0, 0.5).normalized() }));
        scene.lightSystem = lightSystem;

        Image tex = loadTexture("tex.jpeg");
        TextureId texId = scene.textures.move(std::move(tex));

        // scene.geoms.push_back(PlainSphere(Vector3(-0.2, 0.0, 0.0), 0.25, shade1));
        scene.geoms.push_back(Sphere(Vector3(0.3, 0.0, -0.4), 0.25, shade1, texId));
        /*scene.geoms.push_back(Triangle(Vector3(-1.0, 1.0, -0.4), Vector3(-1.0, -1.0, -0.7),
                                       Vector3(2.0, 1.0, -0.3), shadetri));
         */
        scene.geoms.push_back(PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(2.0, -1.0, 1.0),
                                            Vector3(2.0, 0.0, -1.0), shadewall));
        scene.geoms.push_back(PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(-2.0, -1.0, 1.0),
                                            Vector3(2.0, -1.0, 1.0), shadewall));
        scene.geoms.push_back(Sphere(Vector3(-0.2, -0.1, 0), 0.25, shade1, texId));

        // sp.transform = toHomo(scale3(1.0, 0.5, 1.0));
        // sp.itransform = *invertMatrix4x4(sp.transform);

        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.0, 1.4), basis, 1.0);
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
