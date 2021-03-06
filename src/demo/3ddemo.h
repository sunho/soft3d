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

void instantiateMesh(Scene& scene, Model& model, int i, Material material, std::string tex, Matrix transform = I4x4) {
    Image* texture = nullptr;
    if (!tex.empty()) {
        Image base = loadTexture(tex);
        texture = scene.textures.construct<Image>();
        *texture = base;
    }
    auto triangles = model.generateTriangles(i, material, texture);
    for (auto& tri : triangles) {
        auto va = (transform * tri.vA.pos.expand(1.0f)).homoDiv();
        auto vb = (transform * tri.vB.pos.expand(1.0f)).homoDiv();
        auto vc = (transform * tri.vC.pos.expand(1.0f)).homoDiv();
        tri.vA.pos = va;
        tri.vB.pos = vb;
        tri.vC.pos = vc;
        tri.curve = Triangle3(va, vb, vc);
        auto geom = scene.geoms.construct<Triangle>(tri);
        if (material.medium) {
            geom->alpha = true;
            //geom->material.medium = nullptr;
        }
    }
}

struct VolumeScatter : public App {
    VolumeScatter() {
    }
    ~VolumeScatter() {
    }

    void init(Runtime& engine, Scene& scene) override {
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(-4, 7, 26), basis, 1.0);
        scene.camera.lookAt(Vector3(0, 0.0, 0), Vector3(0, 1, 0));
        Image envMap = loadTexture("resources/env-sunset.png");
        Image* envMapId = scene.textures.construct<Image>(envMap);
        scene.environmentMap = envMapId;
        //scene.lights.construct<DirectionalLight>(Vector3(1,1,1)*4.0f, Vector3(-1.0, -1.0, -1.0).normalized(), 5.0f);
        auto l = scene.lights.construct<DiskLight>(Vector3(1, 1, 1)*50, Vector3(-4, 7, 26), 2.5f);
        l->lookAt(Vector3(0,0,0));
        //auto l2 = scene.lights.construct<DiskLight>(Vector3(1, 1, 1)*10, Vector3(0, 13, 0), 5f);
        //l2->lookAt(Vector3(0,0,0));
            //        lightSystem.lights.move({ AreaLight{ 0.5, Vector3(-0.2, 0.3, 1.0), Vector3(0.0, 0.4, 0.0),
//                                             Vector3(0.4, 0.0, 0.0) } });
        //scene.lights.construct<DirectionalLight>(Vector3(0x3ff2ce)*3.0f, Vector3(-1.0, 0.0, -1.0).normalized(), 5.0f);
        //scene.lights.construct<DirectionalLight>(Vector3(0xff334e)*3.0f, Vector3(1.0, 0.0, -1.0).normalized(), 5.0f);

        Material material1 = {
            .diffuse = Vector3(0x6e6e6e), .brdf = scene.brdfs.construct<DielectricBRDF>(1.3),
            .medium = new HomoMedium((Vector3(1,1,1)-Vector3(0x70492f))*0.9+0.1f*Vector3(0x70492f),
                                     0.1f*(Vector3(0x70492f)), new HenyeyGreenstein(0.5))
        };
        Material material2 = {
            .diffuse = Vector3(1,1,1), .brdf = scene.brdfs.construct<GlossyBRDF>(0.3, 0.1)
        };
        Material spoonMat = {
            .diffuse = Vector3(0.7,0.7,0.7), .brdf = scene.brdfs.construct<GlossyBRDF>(0.9, 0.0f)
           };
        Material material3 = { .diffuse = Vector3(0.7, 0.7, 0.7) };

        Material defaultMat = { .diffuse = Vector3(0.9, 0.9, 0.9),
            .brdf = scene.brdfs.construct<LambertianBRDF>() };

        std::map<std::string, Material> materialMap;
        materialMap["cafe"] = material1;
        materialMap["porcelana"] = material2;
        materialMap["Material"] = material2;
        materialMap["None"] = material2;
        materialMap["Material.002"] = spoonMat;
        Model cup = loadObj("resources/cup.obj");
        for (int i = 0; i < cup.meshes.size(); ++i) {
            Material current;
            auto it = materialMap.find(cup.meshes[i].material);
            if (it != materialMap.end()) {
                current = it->second;
            } else {
                current = defaultMat;
            }
            instantiateMesh(scene, cup, i, current, "");
        }
        // instantiateMesh(scene, model2, 0, materialglass, "");
        /*Model model = loadObj("resources/anime.obj");
        instantiateMesh(scene, model, 0, material3, "resources/body_v.jpeg", scale3(0.5,0.5,0.5));
        instantiateMesh(scene, model, 1, material3, "resources/face_c.jpeg", scale3(0.5,0.5,0.5));
        instantiateMesh(scene, model, 2, material3, "resources/hair_c.jpeg", scale3(0.5,0.5,0.5));
        instantiateMesh(scene, model, 3, material3, "resources/acce_c.jpeg", scale3(0.5,0.5,0.5));*/
    }

    void update(Runtime& engine, Scene& scene, double dt) override {
    }
};

struct MetalWolf : public App {
    MetalWolf() {
    }
    ~MetalWolf() {
    }

    void init(Runtime& engine, Scene& scene) override {
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(1.3, 1.3, 1.3), basis, 1.0);
        scene.camera.lookAt(Vector3(0, 0.3, 0), Vector3(0, 1, 0));
        Image envMap = loadTexture("resources/env-midday.png");
        Image* envMapId = scene.textures.construct<Image>(envMap);
        scene.environmentMap = envMapId;
        scene.lights.construct<DirectionalLight>(Vector3(3.0f, 5.0f, 5.0f),
                                                 Vector3(1.0, 1.0, 1.0).normalized(), 5.0f);

        Material material1 = { .diffuse = Vector3(0.7, 0.7, 0.7),
                               .brdf = scene.brdfs.construct<AntPhongBRDF>(0.7, 500, 500) };

        Material material3 = { .diffuse = Vector3(0.7, 0.7, 0.7), .brdf = scene.brdfs.construct<LambertianBRDF>()};

        Material material2 = {
            .diffuse = Vector3(0.3, 0.3, 0.3),
            .brdf = scene.brdfs.construct<AntPhongBRDF>(0.3, 50, 50)
        };

        //material2.brdf  = AntPhongBRDF{ .Rs = 0.8, .nu = 500, .nv = 500 };
        Material materialwall = { .diffuse = Vector3(0.4, 0.4, 0.4),
                                  //.idealReflect = Vector3(0.2, 0.2, 0.2),
                                  .phong = 100.0 };
        Material materialglass2 = { 
                                    .ignoreShadow = false };
        // scene.geoms.add({ PlainSphere(Vector3(-0.5, 0.3, 0.4), 0.25, materialglass2) });
        //scene.geoms.add({ PlainSphere(Vector3(0.0, 0.98, 0.6), 0.32, materialglass) });

        Float sz = -0.3;

        Model wolf = loadObj("resources/wolf2.obj");
        instantiateMesh(scene, wolf, 0, material1, "");
        Model plane = loadObj("resources/plane.obj");
        instantiateMesh(scene, plane, 0, material2, "");
        // instantiateMesh(scene, model2, 0, materialglass, "");
        /*Model model = loadObj("resources/anime.obj");
        instantiateMesh(scene, model, 0, material3, "resources/body_v.jpeg", scale3(0.5,0.5,0.5));
        instantiateMesh(scene, model, 1, material3, "resources/face_c.jpeg", scale3(0.5,0.5,0.5));
        instantiateMesh(scene, model, 2, material3, "resources/hair_c.jpeg", scale3(0.5,0.5,0.5));
        instantiateMesh(scene, model, 3, material3, "resources/acce_c.jpeg", scale3(0.5,0.5,0.5));*/
    }

    void update(Runtime& engine, Scene& scene, double dt) override {
    }
};


struct Room : public App {
    Room() {
    }
    ~Room() {
    }

    void init(Runtime& engine, Scene& scene) override {
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(0.0, 0.65, 1.5), basis, 1.0);
        /*lightSystem.lights.move(
            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));*/
        scene.lights.construct<AreaLight>(Vector3(4.0f,4.0f,4.0f), Vector3(-0.2, 1.12, 0.2), Vector3(0.4, 0.0, 0.0),
                                          Vector3(0.0, 0.0, 0.4));
        Image envMap = loadTexture("resources/env-midday.png");
        Image* envMapId = scene.textures.construct<Image>(envMap);
        scene.environmentMap = envMapId;

        Material material1 = { .diffuse = Vector3(0.5, 1.0, 0.5)

                               };

        Material material2 = {
            .diffuse = Vector3(0.9, 0.9, 0.9),
            .phong = 100.0
        };

        //material2.brdf  = AntPhongBRDF{ .Rs = 0.8, .nu = 500, .nv = 500 };
        Material materialwall = { .diffuse = Vector3(0.4, 0.4, 0.4),
                                  //.idealReflect = Vector3(0.2, 0.2, 0.2),
                                  .phong = 100.0 };
        Material materialglass2 = { 
                                    .ignoreShadow = false };
        // scene.geoms.add({ PlainSphere(Vector3(-0.5, 0.3, 0.4), 0.25, materialglass2) });
        //scene.geoms.add({ PlainSphere(Vector3(0.0, 0.98, 0.6), 0.32, materialglass) });

        Float sz = -0.3;

        Model model = loadObj("resources/room.obj");
        instantiateMesh(scene, model, 0, material1, "resources/cube.jpg");
         scene.geoms.construct<Sphere>(Vector3(0.1, 0.35, 0.05), 0.2, material2, nullptr);

        // instantiateMesh(scene, model2, 0, materialglass, "");
    }

    void update(Runtime& engine, Scene& scene, double dt) override {
    }
};

//struct AnimeLoad : public App {
//    AnimeLoad() {
//    }
//    ~AnimeLoad() {
//    }
//
//    void init(Runtime& engine, Scene& scene) override {
//        Basis basis;
//        basis.u = Vector3(1.0, 0.0, 0.0);
//        basis.v = Vector3(0.0, 1.0, 0.0);
//        basis.w = Vector3(0.0, 0.0, 1.0);
//        scene.camera = Camera(Vector3(0.0, 0.6, 2.0), basis, 1.0);
//        LightSystem lightSystem{};
//        lightSystem.ambientIntensity = 0.7;
//        /*lightSystem.lights.move(
//            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));*/
//        lightSystem.lights.move({ AreaLight{ 0.5, Vector3(-0.2, 0.3, 1.0), Vector3(0.0, 0.4, 0.0),
//                                             Vector3(0.4, 0.0, 0.0) } });
//        scene.lightSystem = lightSystem;
//
//        Material material1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
//                               .ambient = Vector3(0.5, 1.0, 0.5),
//                               .specular = Vector3(0.3, 0.3, 0.3),
//
//                               .phong = 100.0 };
//
//        Material material2 = { .diffuse = Vector3(0.5, 0.5, 0.5),
//                               .ambient = Vector3(0.5, 0.5, 0.5),
//                               .specular = Vector3(0.3, 0.3, 0.3),
//                               .phong = 100.0,
//                               .idealReflect = Vector3(0.3, 0.3, 0.3)
//        };
//
//        Material materialwall = { .diffuse = Vector3(0.4, 0.4, 0.8),
//                                  .ambient = Vector3(0.8, 0.8, 0.8),
//                                  .specular = Vector3(0.3, 0.3, 0.3),
//                                  //.idealReflect = Vector3(0.2, 0.2, 0.2),
//                                  .phong = 100.0 };
//        Material materialglass = { .refractIndex = 1.3f,
//                                   .refractReflectance = Vector3(0.0, 0.0, -0.2),
//                                   .ignoreShadow = false };
//        Material materialglass2 = { .refractIndex = 1.6f,
//                                    .refractReflectance = Vector3(0.0, 0.0, -0.2),
//                                    .ignoreShadow = false };
//        scene.environmentMap = scene.textures.move(loadTexture("resources/env.jpg"));
//        scene.geoms.add({ PlainSphere(Vector3(0.4, 0.15, 0.4), 0.2, materialglass2) });
//        scene.geoms.add({ PlainSphere(Vector3(-0.5, 0.3, 0.4), 0.25, materialglass2) });
//        //scene.geoms.add({ PlainSphere(Vector3(0.0, 0.98, 0.6), 0.32, materialglass) });
//
//        Float sz = -0.3;
//        scene.geoms.add({ PlainTriangle(Vector3(-2.0, 2.0, sz), Vector3(-2.0, -2.0, sz),
//                                        Vector3(2.0, 2.0, sz), materialwall) });
//        scene.geoms.add({ PlainTriangle(Vector3(-2.0, -2.0, sz), Vector3(2.0, -2.0, sz),
//                                        Vector3(2.0, 2.0, sz), materialwall) });
//
//        Model model = loadObj("resources/anime.obj");
//        Model model2 = loadObj("resources/glass.obj");
//        instantiateMesh(scene, model, 0, material1, "resources/body_v.jpeg");
//        instantiateMesh(scene, model, 1, material1, "resources/face_c.jpeg");
//        instantiateMesh(scene, model, 2, material1, "resources/hair_c.jpeg");
//        instantiateMesh(scene, model, 3, material1, "resources/acce_c.jpeg");
//        // instantiateMesh(scene, model2, 0, materialglass, "");
//    }
//
//    void update(Runtime& engine, Scene& scene, double dt) override {
//    }
//};
//
//struct ObjLoad : public App {
//    ObjLoad() {
//    }
//    ~ObjLoad() {
//    }
//
//    void init(Runtime& engine, Scene& scene) override {
//        Basis basis;
//        basis.u = Vector3(1.0, 0.0, 0.0);
//        basis.v = Vector3(0.0, 1.0, 0.0);
//        basis.w = Vector3(0.0, 0.0, 1.0);
//        scene.camera = Camera(Vector3(0.0, 0.3, 1.5), basis, 1.0);
//        LightSystem lightSystem{};
//        lightSystem.ambientIntensity = 0.3;
//        /*lightSystem.lights.move(
//            std::move(DirectionalLight{ 1.2, Vector3(-0.5, 0.5, 0.5).normalized() }));*/
//        lightSystem.lights.move({ AreaLight{ 1.2, Vector3(-0.05, 0.35, 1.0), Vector3(0.0, 0.1, 0.0),
//                                             Vector3(0.1, 0.0, 0.0) } });
//        lightSystem.lights.move({ DirectionalLight{ 0.6, Vector3(0.0, 0.0, 1.0) } });
//        scene.lightSystem = lightSystem;
//
//        Material material1 = { .diffuse = Vector3(0.5, 1.0, 0.5),
//                               .ambient = Vector3(0.5, 1.0, 0.5),
//                               .specular = Vector3(0.3, 0.3, 0.3),
//                               //.idealReflect = Vector3(0.0, 0.0, 0.0),
//                               .phong = 100.0 };
//
//        Material materialwall = { .diffuse = Vector3(0.4, 0.4, 0.4),
//                                  .ambient = Vector3(0.8, 0.8, 0.8),
//                                  .specular = Vector3(0.3, 0.3, 0.3),
//                                  .phong = 100.0,
//                                  .idealReflect = Vector3(0.2, 0.2, 0.2) };
//        Material materialglass = { .refractIndex = 1.5f,
//                                   .refractReflectance = Vector3(0.0, 0.0, -0.3) };
//
//        // scene.geoms.add({ PlainSphere(Vector3(0.0, 0.15, 0.7), 0.1, materialglass) });
//        Float sz = -0.3;
//        scene.geoms.add({ PlainTriangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0, sz),
//                                        Vector3(2.0, 2.0, sz), materialwall) });
//        scene.geoms.add({ PlainTriangle(Vector3(-1.0, -1.0, sz), Vector3(1.0, -2.0, sz),
//                                        Vector3(2.0, 2.0, sz), materialwall) });
//
//        Model model = loadObj("resources/tree.obj");
//        instantiateMesh(scene, model, 0, material1, "resources/tree_base.png");
//    }
//
//    void update(Runtime& engine, Scene& scene, double dt) override {
//    }
//};
//
//struct SphereRayTrace : public App {
//    SphereRayTrace() {
//    }
//    ~SphereRayTrace() {
//    }
//
//    void init(Runtime& engine, Scene& scene) override {
//        Material material1 {
//            .diffuse = Vector3(0x97f291),
//            .ambient = Vector3(0x97f291),
//            .specular = Vector3(0.3, 0.3, 0.3),
//            .phong = 100.0,
//        };
//        Material material2 {
//            .diffuse = Vector3(0x9dcdfc),
//            .ambient = Vector3(0x9dcdfc),
//            .specular = Vector3(0.3, 0.3, 0.3),
//            .phong = 100.0,
//        };
//        Material materialwall { .diffuse = Vector3(0.3, 0.3, 0.3),
//                                  .ambient = Vector3(0.8, 0.8, 0.8),
//                                  .specular = Vector3(0.3, 0.3, 0.3),
//                                  .phong = 100.0,
//                                  .idealReflect = Vector3(0.2, 0.2, 0.2),
//        };
//        LightSystem lightSystem;
//        lightSystem.ambientIntensity = 0.4;
//        lightSystem.lights.move({ AreaLight{ 0.9, Vector3(-0.3, 0.8, 0.6), Vector3(0.0, 0.3, 0.0),
//                                                   Vector3(0.3, 0.0, 0.3) } });
//        scene.lightSystem = lightSystem;
//
//        Image tex = loadTexture("resources/sphere.jpeg");
//        TextureId texId = scene.textures.move(std::move(tex));
//
//        scene.geoms.add({ PlainSphere(Vector3(0.3, 0.0, -0.4), 0.25, material1) });
//        scene.geoms.add({ PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(2.0, -1.0, 1.0),
//                                        Vector3(2.0, 0.0, -1.0), materialwall) });
//        scene.geoms.add({ PlainTriangle(Vector3(-2.0, 0.0, -1.0), Vector3(-2.0, -1.0, 1.0),
//                                        Vector3(2.0, -1.0, 1.0), materialwall) });
//        scene.geoms.add({ PlainSphere(Vector3(-0.2, -0.1, 0), 0.25, material2) });
//        Basis basis;
//        basis.u = Vector3(1.0, 0.0, 0.0);
//        basis.v = Vector3(0.0, 1.0, 0.0);
//        basis.w = Vector3(0.0, 0.0, 1.0);
//        scene.camera = Camera(Vector3(0.0, 0.0, 1.4), basis, 1.0);
//        
//          //scene.environmentMap = scene.textures.move(loadTexture("resources/env2.jpg"));
//    }
//
//    void update(Runtime& engine, Scene& scene, double dt) override {
//        Camera& camera = scene.camera;
//        if (engine.pressed(GLFW_KEY_W)) {
//            camera.e.z() -= 0.05;
//        }
//        if (engine.pressed(GLFW_KEY_S)) {
//            camera.e.z() += 0.05;
//        }
//        if (engine.pressed(GLFW_KEY_A)) {
//            camera.e.x() -= 0.05;
//        }
//        if (engine.pressed(GLFW_KEY_D)) {
//            camera.e.x() += 0.05;
//        }
//    }
//};
//
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
        rt.renderHdr(d, "fb.hdr");                                                                    \
    }
