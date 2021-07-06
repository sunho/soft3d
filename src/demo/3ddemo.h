#include <soft3d/backend/rtcpu/renderer.h>
#include <soft3d/backend/zcpu/renderer.h>
#include <soft3d/common/curve.h>
#include <soft3d/common/image.h>
#include <soft3d/common/scene.h>
#include <soft3d/common/transform.h>
#include <soft3d/engine/engine.h>
#include <soft3d/engine/loader.h>

#include <chrono>
#include <thread>

static EngineConfig engineConf = { .width = 500,
                                   .height = 500,
                                   .fps = 60,
                                   /*  .aaProfile = AAProfile {
                                         .filter = boxFilter1(1),
                                     },*/
                                   .renderer = Backend::RTCPU };
struct AnimeLoad : public App {
    Model model;
    Model model2;
    AnimeLoad() {
    }
    ~AnimeLoad() {
    }

    void init(Engine& engine, Scene& scene) override {
        model = loadObj("resources/anime.obj");
        model2 = loadObj("resources/glass.obj");
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.4, 1.5), basis, 1.0);
        LightSystem lightSystem{};
        lightSystem.ambientIntensity = 0.6;
        /*lightSystem.lights.move(
            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));*/
        lightSystem.lights.move(std::move(AreaLight{
            0.6, Vector3(-0.05, 0.35, 1.0), Vector3(0.0, 0.1, 0.0), Vector3(0.1, 0.0, 0.0) }));
        scene.lightSystem = lightSystem;

        Shade shade1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
                         .ambient = Vector3(0.5, 1.0, 0.5),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .idealReflect = Vector3(0.0, 0.0, 0.0),
                         .phong = 1.0 };

        Shade shade2 = { .diffuse = Vector3(0.5, 0.5, 1.0),
                         .ambient = Vector3(0.5, 0.5, 1.0),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .idealReflect = Vector3(0.2, 0.2, 0.2),
                         .phong = 100.0 };
        Shade shadewall = { .diffuse = Vector3(0.4, 0.4, 0.4),
                            .ambient = Vector3(0.8, 0.8, 0.8),
                            .specular = Vector3(0.3, 0.3, 0.3),
                            .idealReflect = Vector3(0.2, 0.2, 0.2),
                            .phong = 100.0 };
        Shade shadeglass = { .refractIndex = 1.34f, .refractReflectance = Vector3(0.5, 0.5, 0.5) };

        scene.geoms.add(PlainSphere(Vector3(0.3, 0.15, 0.2), 0.15, shadeglass));
        scene.geoms.add(PlainSphere(Vector3(-0.4, 0.3, 0.2), 0.2, shadeglass));
        scene.geoms.add(PlainSphere(Vector3(0.3, 0.65, 0.4), 0.1, shadeglass));
        scene.geoms.add(PlainSphere(Vector3(0.0, 0.75, 0.2), 0.2, shadeglass));

        Float sz = -0.3;
        scene.geoms.add(PlainTriangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0, sz),
                                      Vector3(2.0, 2.0, sz), shadewall));
        scene.geoms.add(PlainTriangle(Vector3(-1.0, -1.0, sz), Vector3(1.0, -2.0, sz),
                                      Vector3(2.0, 2.0, sz), shadewall));

        const auto loadMesh = [&](Model& model, int i, Shade shade, std::string tex) {
            TextureId baseId = 0;
            if (!tex.empty()) {
                Image base = loadTexture(tex);
                baseId = scene.textures.move(std::move(base));
            }
            for (auto tri : model.meshes[i].data) {
                auto tt = Triangle({ tri.a, tri.nA, tri.tA }, { tri.b, tri.nB, tri.tB },
                                   { tri.c, tri.nC, tri.tC }, shade);
                tt.texture = baseId;
                scene.geoms.add(tt);
            }
        };
        loadMesh(model, 0, shade1, "resources/body_v.jpeg");
        loadMesh(model, 1, shade1, "resources/face_c.jpeg");
        loadMesh(model, 2, shade1, "resources/hair_c.jpeg");
        loadMesh(model, 3, shade1, "resources/acce_c.jpeg");
        // loadMesh(model2, 0, shadeglass, "");
    }

    void update(Engine& engine, Scene& scene, double dt) override {
    }
};

struct ObjLoad : public App {
    Model model;

    ObjLoad() {
    }
    ~ObjLoad() {
    }

    void init(Engine& engine, Scene& scene) override {
        Image base = loadTexture("resources/tree_base.png");
        TextureId baseId = scene.textures.move(std::move(base));
        Image normal = loadTexture("resources/tree_normal.png");
        TextureId normalId = scene.textures.move(std::move(normal));
        model = loadObj("resources/tree.obj");
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.0, 1.5), basis, 1.0);
        LightSystem lightSystem{};
        lightSystem.ambientIntensity = 0.3;
        /*lightSystem.lights.move(
            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));*/
        lightSystem.lights.move(std::move(AreaLight{
            1.2, Vector3(-0.05, 0.35, 1.0), Vector3(0.0, 0.1, 0.0), Vector3(0.1, 0.0, 0.0) }));
        lightSystem.lights.move(std::move(DirectionalLight{ 0.6, Vector3(0.0, 0.0, 1.0) }));
        scene.lightSystem = lightSystem;

        Shade shade1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
                         .ambient = Vector3(0.5, 1.0, 0.5),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .idealReflect = Vector3(0.0, 0.0, 0.0),
                         .phong = 100.0 };

        Shade shade2 = { .diffuse = Vector3(0.5, 0.5, 1.0),
                         .ambient = Vector3(0.5, 0.5, 1.0),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .idealReflect = Vector3(0.2, 0.2, 0.2),
                         .phong = 100.0 };
        Shade shadewall = { .diffuse = Vector3(0.4, 0.4, 0.4),
                            .ambient = Vector3(0.8, 0.8, 0.8),
                            .specular = Vector3(0.3, 0.3, 0.3),
                            .idealReflect = Vector3(0.2, 0.2, 0.2),
                            .phong = 100.0 };
        Shade shadeglass = { .refractIndex = 1.5f, .refractReflectance = Vector3(-1.0, -1.0, 0.6) };

        scene.geoms.add(PlainSphere(Vector3(0.0, 0.15, 0.6), 0.1, shadeglass));
        Float sz = -0.3;
        scene.geoms.add(PlainTriangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0, sz),
                                      Vector3(2.0, 2.0, sz), shadewall));
        scene.geoms.add(PlainTriangle(Vector3(-1.0, -1.0, sz), Vector3(1.0, -2.0, sz),
                                      Vector3(2.0, 2.0, sz), shadewall));

        for (auto tri : model.meshes[0].data) {
            auto tt = Triangle({ tri.a, tri.nA, tri.tA }, { tri.b, tri.nB, tri.tB },
                               { tri.c, tri.nC, tri.tC }, shade1);
            tt.texture = baseId;
            tt.normalMap = normalId;
            scene.geoms.add(tt);
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
        Shade shade1 = {
            .diffuse = Vector3(0x9dcdfc),
            .ambient = Vector3(0x9dcdfc),
            .specular = Vector3(0.1, 0.1, 0.1),
            .phong = 100.0,
        };
        Shade shade2 = { .diffuse = Vector3(0xfb9dfc),
                         .ambient = Vector3(0xfb9dfc),
                         .specular = Vector3(0.3, 0.3, 0.3),
                         .phong = 100.0 };
        Shade shadewall = { .diffuse = Vector3(0.3, 0.3, 0.3),
                            .ambient = Vector3(0.8, 0.8, 0.8),
                            .specular = Vector3(0.3, 0.3, 0.3),
                            .idealReflect = Vector3(0.2, 0.2, 0.2),
                            .phong = 100.0 };
        Shade shadetri = { .diffuse = Vector3(0xd3e4f5),
                           .ambient = Vector3(0xd3e4f5),
                           .specular = Vector3(0.3, 0.3, 0.3),
                           .idealReflect = Vector3(0.2, 0.2, 0.2),
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

        Image tex = loadTexture("resources/sphere.jpeg");
        TextureId texId = scene.textures.move(std::move(tex));

        // scene.geoms.push_back(PlainSphere(Vector3(-0.2, 0.0, 0.0), 0.25, shade1));
        scene.geoms.add(Sphere(Vector3(0.3, 0.0, -0.4), 0.25, shade1, texId));
        /*scene.geoms.push_back(Triangle(Vector3(-1.0, 1.0, -0.4), Vector3(-1.0, -1.0, -0.7),
                                       Vector3(2.0, 1.0, -0.3), shadetri));
         */
        scene.geoms.add(PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(2.0, -1.0, 1.0),
                                      Vector3(2.0, 0.0, -1.0), shadewall));
        scene.geoms.add(PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(-2.0, -1.0, 1.0),
                                      Vector3(2.0, -1.0, 1.0), shadewall));
        scene.geoms.add(Sphere(Vector3(-0.2, -0.1, 0), 0.25, shade1, texId));

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

#define RENDER3D(demo)                                                                             \
    {                                                                                              \
        Engine engine(engineConf);                                                                 \
        App* d = new demo;                                                                         \
        engine.render(d, "fb.png");                                                                \
    }
