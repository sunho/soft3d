#include <soft3d/image/image.h>
#include <soft3d/math/curve.h>
#include <soft3d/math/transform.h>
#include <soft3d/runtime/loader.h>
#include <soft3d/runtime/runtime.h>
#include <soft3d/scene/scene.h>

#include <chrono>
#include <thread>

static RuntimeConfig engineConf = { .width = 500,
                                    .height = 500,
                                    .fps = 60,
                                    /*  .aaProfile = AAProfile {
                                          .filter = boxFilter1(1),
                                      },*/
                                    .renderer = Backend::RTCPU };

void instantiateMesh(Scene& scene, Model& model, int i, Material material, std::string tex) {
    TextureId texId = 0;
    if (!tex.empty()) {
        Image base = loadTexture(tex);
        texId = scene.textures.move(std::move(base));
    }
    auto triangles = model.generateTriangles(i, material, texId);
    for (auto& tri : triangles) {
        scene.geoms.add(tri);
    }
}

struct AnimeLoad : public App {
    AnimeLoad() {
    }
    ~AnimeLoad() {
    }

    void init(Runtime& engine, Scene& scene) override {
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.4, 2.0), basis, 1.0);
        LightSystem lightSystem{};
        lightSystem.ambientIntensity = 0.5;
        /*lightSystem.lights.move(
            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));*/
        lightSystem.lights.move(std::move(AreaLight{
            0.7, Vector3(-0.05, 0.6, 1.0), Vector3(0.0, 0.1, 0.0), Vector3(0.1, 0.0, 0.0) }));
        scene.lightSystem = lightSystem;

        Material material1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
                               .ambient = Vector3(0.5, 1.0, 0.5),
                               .specular = Vector3(0.3, 0.3, 0.3),

                               .phong = 1.0 };

        Material material2 = { .diffuse = Vector3(0.5, 0.5, 0.5),
                               .ambient = Vector3(0.5, 0.5, 0.5),
                               .specular = Vector3(0.3, 0.3, 0.3),
                               .idealReflect = Vector3(0.3, 0.3, 0.3),
                               .phong = 100.0 };

        Material materialwall = { .diffuse = Vector3(0.4, 0.4, 0.4),
                                  .ambient = Vector3(0.8, 0.8, 0.8),
                                  .specular = Vector3(0.3, 0.3, 0.3),
                                  //.idealReflect = Vector3(0.3, 0.3, 0.3),
                                  .phong = 100.0 };
        Material materialglass = { .refractIndex = 1.5f,
                                   .refractReflectance = Vector3(0.0, 0.0, -0.2) };
        scene.environmentMap = scene.textures.move(loadTexture("resources/env.jpg"));
        scene.geoms.add(PlainSphere(Vector3(0.3, 0.15, 0.4), 0.15, material2));
        scene.geoms.add(PlainSphere(Vector3(-0.4, 0.3, 0.4), 0.2, materialglass));
        // scene.geoms.add(PlainSphere(Vector3(0.0, 0.75, 0.5), 0.3, materialglass));

        Float sz = -0.3;
        scene.geoms.add(PlainTriangle(Vector3(-2.0, 2.0, sz), Vector3(-2.0, -2.0, sz),
                                      Vector3(2.0, 2.0, sz), materialwall));
        scene.geoms.add(PlainTriangle(Vector3(-2.0, -2.0, sz), Vector3(2.0, -2.0, sz),
                                      Vector3(2.0, 2.0, sz), materialwall));

        Model model = loadObj("resources/anime.obj");
        Model model2 = loadObj("resources/glass.obj");
        instantiateMesh(scene, model, 0, material1, "resources/body_v.jpeg");
        instantiateMesh(scene, model, 1, material1, "resources/face_c.jpeg");
        instantiateMesh(scene, model, 2, material1, "resources/hair_c.jpeg");
        instantiateMesh(scene, model, 3, material1, "resources/acce_c.jpeg");
        // instantiateMesh(scene, model2, 0, materialglass, "");
    }

    void update(Runtime& engine, Scene& scene, double dt) override {
    }
};

struct ObjLoad : public App {
    ObjLoad() {
    }
    ~ObjLoad() {
    }

    void init(Runtime& engine, Scene& scene) override {
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
        lightSystem.lights.move(std::move(DirectionalLight{ 0.4, Vector3(0.0, 0.0, 1.0) }));
        scene.lightSystem = lightSystem;

        Material material1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
                               .ambient = Vector3(0.5, 1.0, 0.5),
                               .specular = Vector3(0.3, 0.3, 0.3),
                               .idealReflect = Vector3(0.0, 0.0, 0.0),
                               .phong = 100.0 };

        Material materialwall = { .diffuse = Vector3(0.4, 0.4, 0.4),
                                  .ambient = Vector3(0.8, 0.8, 0.8),
                                  .specular = Vector3(0.3, 0.3, 0.3),
                                  .idealReflect = Vector3(0.2, 0.2, 0.2),
                                  .phong = 100.0 };
        Material materialglass = { .refractIndex = 1.5f,
                                   .refractReflectance = Vector3(0.0, 0.0, -0.3) };

        scene.geoms.add(PlainSphere(Vector3(0.0, 0.15, 0.7), 0.1, materialglass));
        Float sz = -0.3;
        scene.geoms.add(PlainTriangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0, sz),
                                      Vector3(2.0, 2.0, sz), materialwall));
        scene.geoms.add(PlainTriangle(Vector3(-1.0, -1.0, sz), Vector3(1.0, -2.0, sz),
                                      Vector3(2.0, 2.0, sz), materialwall));

        Model model = loadObj("resources/tree.obj");
        instantiateMesh(scene, model, 0, material1, "resources/tree_base.png");
    }

    void update(Runtime& engine, Scene& scene, double dt) override {
    }
};

struct SphereRayTrace : public App {
    SphereRayTrace() {
    }
    ~SphereRayTrace() {
    }

    void init(Runtime& engine, Scene& scene) override {
        Material material1 = {
            .diffuse = Vector3(0x9dcdfc),
            .ambient = Vector3(0x9dcdfc),
            .specular = Vector3(0.1, 0.1, 0.1),
            .phong = 100.0,
        };
        Material materialwall = { .diffuse = Vector3(0.3, 0.3, 0.3),
                                  .ambient = Vector3(0.8, 0.8, 0.8),
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

        scene.geoms.add(Sphere(Vector3(0.3, 0.0, -0.4), 0.25, material1, texId));
        scene.geoms.add(PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(2.0, -1.0, 1.0),
                                      Vector3(2.0, 0.0, -1.0), materialwall));
        scene.geoms.add(PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(-2.0, -1.0, 1.0),
                                      Vector3(2.0, -1.0, 1.0), materialwall));
        scene.geoms.add(Sphere(Vector3(-0.2, -0.1, 0), 0.25, material1, texId));
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.0, 1.4), basis, 1.0);
    }

    void update(Runtime& engine, Scene& scene, double dt) override {
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
        Runtime rt(engineConf);                                                                    \
        App* d = new demo;                                                                         \
        rt.run(d);                                                                                 \
    }

#define RENDER3D(demo)                                                                             \
    {                                                                                              \
        Runtime rt(engineConf);                                                                    \
        App* d = new demo;                                                                         \
        rt.render(d, "fb.png");                                                                    \
    }
