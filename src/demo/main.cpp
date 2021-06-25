#include <focg/curve.h>
#include <focg/screen.h>
#include <focg/loader.h>
#include <focg/raytracer.h>
#include <chrono>
#include <thread>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

constexpr size_t WIDTH = 500;
constexpr size_t HEIGHT = 500;

struct KeyboardState {
    GLFWwindow* window;
    bool pressed(int keycode) {
        int state = glfwGetKey(window, keycode);
        return (state == GLFW_PRESS);
    }

};

struct PrimitiveDraw {
    void render(Screen &screen, KeyboardState& key) {
        begin(screen, key);
        for (int i = 0; i < WIDTH; ++i) {
           for (int j = 0; j < HEIGHT; ++j) {
               Vector2 pos(i,j);
               frag(screen, pos);
           }
        }
        end(screen, key);
    }

    virtual void begin(Screen &screen, KeyboardState& key) { }
    virtual void end(Screen &screen, KeyboardState& key) { }
    virtual void frag(Screen& screen, Vector2 pos) = 0;
};

struct Light {
    Float intensity;
    Vector3 v;
};

struct ObjLoad : public PrimitiveDraw {
    Scene scene;
    Model model;
    Camera camera;
    Float ambientI;
    RayObject* mirror;
    RayObject* mirror2;
    std::vector<Light> lights;
    
    ObjLoad() {
        model = loadObj("model.obj");
        
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        camera = Camera(Vector3(-0.5, 0.0, 2.0), basis);

        Shade shade1 = {
            .diffuse = Vector3(0.5,1.0,0.5),
            .ambient = Vector3(0.5,1.0,0.5),
            .specular = Vector3(0.3, 0.3, 0.3),
            .p = 100.0
        };
    
        Shade shade2 = {
            .diffuse = Vector3(0.5,0.5,1.0),
            .ambient = Vector3(0.5,0.5,1.0),
            .specular = Vector3(0.3, 0.3, 0.3),
            .p = 100.0
        };
        Shade shade3 = {
            .diffuse = Vector3(0.7,0.7,0.7),
            .ambient = Vector3(0.7,0.7,0.7),
            .specular = Vector3(0.3, 0.3, 0.3),
            .p = 100.0
        };
                Float sz = -0.3;
        auto m = std::make_unique<Triangle3>(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0,  sz), Vector3(2.0, 2.0,  sz),  shade3);
        mirror = m.get();
        scene.addObject(0, std::move(m));
        auto m2 = std::make_unique<Triangle3>(Vector3(-1.0, -1.0,  sz), Vector3(1.0, -2.0, sz), Vector3(2.0, 2.0,  sz),  shade3);
        mirror2 = m2.get();
        scene.addObject(1, std::move(m2));
        scene.addObject(2, std::make_unique<Sphere>(Vector3(0.0, 0.0, -0.15), 0.10, shade2));
        
        
        int i = 3;
        for(auto tri : model.meshes[0].data) {
            //if (tri.a.cross(tri.b).dot(camera.basis.w) > 0.0) {
                 scene.addObject(i++, std::make_unique<Triangle3>(tri.a, tri.b, tri.c,  shade1));
            //}
           
        }
        
        ambientI = 0.2;
        lights.push_back({1.2, Vector3(-0.5, 0.5, 0.5).normalized()});
        lights.push_back({0.3, Vector3(-0.8, -1.0, 0.5).normalized()});
    }

    
    Vector3 rayColor(Ray ray, Float t0, Float t1, int depth = 0) {
        if (depth == 10) {
            return Vector3(0,0,0);
        }
        RayHit hit;
        RayHit hit2;
        const bool test = scene.testRay(ray, t0, t1, hit);
        if (test) {
            Vector3 pixel =  ambientI * hit.shade.ambient;

            for (auto light : lights) {
                if (!scene.testRay(Ray{hit.pos, light.v}, 0.0001, 1.0/0.0, hit2)) {
                    Vector3 h = -1*ray.dir.normalized() + light.v;
                    h.normalize();
                    
                    Float x = std::max(0.0, light.v.dot(hit.normal));
                    Float x2 = std::max(0.0, h.dot(hit.normal));
                    pixel += light.intensity * (x * hit.shade.diffuse + pow(x2, hit.shade.p) * hit.shade.specular);
                }
            }
                Vector3 dir = ray.dir - 2*(ray.dir.dot(hit.normal))*hit.normal;
                pixel += Vector3(0.2,0.2,0.2) * rayColor(Ray{hit.pos, dir}, 0.0001, 1.0/0.0, depth + 1);

            return pixel;
        } else {
            return Vector3(1.0,1.0,1.0);
        }
    }
    
    void frag(Screen &screen, Vector2 pos) override {
        const Ray ray = camera.generateRay(pos, screen);
        screen.setPixel(pos, rayColor(ray, 0.0, 1.0/0.0));
    }
};

struct SphereRayTrace : public PrimitiveDraw {
    Scene scene;
    Camera camera;
    Float ambientI;
    RayObject* mirror;
    
    std::vector<Light> lights;
    
    SphereRayTrace() {
        Shade shade1 = {
            .diffuse = Vector3(0.5,1.0,0.5),
            .ambient = Vector3(0.5,1.0,0.5),
            .specular = Vector3(0.3, 0.3, 0.3),
            .p = 100.0
        };
        Shade shade2 = {
            .diffuse = Vector3(0.5,0.5,1.0),
            .ambient = Vector3(0.5,0.5,1.0),
            .specular = Vector3(0.3, 0.3, 0.3),
            .p = 100.0
        };
        Shade shade3 = {
            .diffuse = Vector3(0.5,0.5,0.5),
            .ambient = Vector3(0.5,0.5,0.5),
            .specular = Vector3(0.3, 0.3, 0.3),
            .p = 100.0
        };
        Shade shade4 = {
            .diffuse = Vector3(0.8,0.8,0.8),
            .ambient = Vector3(0.8,0.8,0.8),
            .specular = Vector3(0.3, 0.3, 0.3),
            .p = 100.0
        };
        scene.addObject(0, std::make_unique<Sphere>(Vector3(-0.2, 0.0, 0.0), 0.25, shade1));
        auto m2 =  std::make_unique<Sphere>(Vector3(0.3, 0.0, -0.4), 0.25, shade2);
                //mirror = m2.get();
        scene.addObject(1, std::move(m2));
        auto m = std::make_unique<Triangle3>(Vector3(-0.5, 1.0, -0.2), Vector3(-0.5, -0.5,  -0.1), Vector3(0.5, 0.5,  -0.5),  shade4);
        //mirror = m.get();
        Float sz = -0.6;
        scene.addObject(2, std::move(m));
        scene.addObject(3, std::make_unique<Triangle3>(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0,  sz), Vector3(2.0, 2.0,  sz),  shade3));
        scene.addObject(4, std::make_unique<Triangle3>(Vector3(-1.0, -1.0,  sz), Vector3(1.0, -2.0, sz), Vector3(2.0, 2.0,  sz),  shade3));
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        camera = Camera(Vector3(-0.5, -0.5, 1.0), basis);
        lights.push_back({1.2, Vector3(-0.5, 0.5, 0.5).normalized()});
        lights.push_back({0.3, Vector3(-0.8, -1.0, 0.5).normalized()});
        lights.push_back({0.3, Vector3(0.8, -1.0, 0.5).normalized()});
        ambientI = 0.2;
    }
    
    void begin(Screen &screen, KeyboardState& key) override {
        if (key.pressed(GLFW_KEY_W)) {
            camera.e.z() -= 0.05;
        }
        if (key.pressed(GLFW_KEY_S)) {
            camera.e.z() += 0.05;
        }
        if (key.pressed(GLFW_KEY_A)) {
             camera.e.x() -= 0.05;
        }
        if (key.pressed(GLFW_KEY_D)) {
             camera.e.x() += 0.05;
        }
    }
    
    Vector3 rayColor(Ray ray, Float t0, Float t1, int depth = 0) {
        if (depth == 10) {
            return Vector3(0,0,0);
        }
        RayHit hit;
        RayHit hit2;
        const bool test = scene.testRay(ray, t0, t1, hit);
        if (test) {
            Vector3 pixel =  ambientI * hit.shade.ambient;

            for (auto light : lights) {
                if (!scene.testRay(Ray{hit.pos, light.v}, 0.0001, 1.0/0.0, hit2)) {
                    Vector3 h = -1*ray.dir.normalized() + light.v;
                    h.normalize();
                    
                    Float x = std::max(0.0, light.v.dot(hit.normal));
                    Float x2 = std::max(0.0, h.dot(hit.normal));
                    pixel += light.intensity * (x * hit.shade.diffuse + pow(x2, hit.shade.p) * hit.shade.specular);
                }
            }
            if (hit.obj->type() == RayObjectType::TRIANGLE3 || hit.obj == mirror) {
                Vector3 dir = ray.dir - 2*(ray.dir.dot(hit.normal))*hit.normal;
                pixel += Vector3(0.2,0.2,0.2) * rayColor(Ray{hit.pos, dir}, 0.0001, 1.0/0.0, depth + 1);
            }
            return pixel;
        } else {
            return Vector3(1.0,1.0,1.0);
        }
    }
    
    void frag(Screen &screen, Vector2 pos) override {
        const Ray ray = camera.generateRay(pos, screen);
        screen.setPixel(pos, rayColor(ray, 0.0, 1.0/0.0));
    }
};

struct DrawSphere : public PrimitiveDraw {
    Sphere sphere;
    Camera camera;
    Vector3 color;
    Vector3 specular;
    Vector3 ambient;
    Float ambientI;
    std::vector<Light> lights;
    
    DrawSphere() {
        sphere = Sphere(Vector3(0.0, 0.0, 0.0), 0.25, Shade());
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        color = Vector3(0.5,1.0,0.5);
        ambient = color;
        specular = Vector3(0.3, 0.3, 0.3);
        camera = Camera(Vector3(-0.5, -0.5, 1.0), basis);
        lights.push_back({1.2, Vector3(-0.5, 0.5, 0.5).normalized()});
        lights.push_back({0.3, Vector3(-0.8, -1.0, 0.5).normalized()});
        lights.push_back({0.3, Vector3(0.8, -1.0, 0.5).normalized()});
        ambientI = 0.2;
    }
    
    void begin(Screen &screen, KeyboardState& key) override {
        if (key.pressed(GLFW_KEY_W)) {
            camera.e.z() -= 0.05;
        }
        if (key.pressed(GLFW_KEY_S)) {
            camera.e.z() += 0.05;
        }
        if (key.pressed(GLFW_KEY_A)) {
             camera.e.x() -= 0.05;
        }
        if (key.pressed(GLFW_KEY_D)) {
             camera.e.x() += 0.05;
        }
    }
    
    void frag(Screen &screen, Vector2 pos) override {
        const Ray ray = camera.generateRay(pos, screen);
        RayHit hit;
        const bool test = sphere.testRay(ray, 0.0, 1.0/0.0, hit);
        if (test) {
            Vector3 pixel =  ambientI * ambient;

            for (auto light : lights) {
                Vector3 h = -1*ray.dir.normalized() + light.v;
                h.normalize();
                
                Float x = std::max(0.0, light.v.dot(hit.normal));
                Float x2 = std::max(0.0, h.dot(hit.normal));
                pixel += light.intensity * (x * color + pow(x2, 100) * specular);
            }
            screen.setPixel(pos, pixel);
        } else {
            screen.setPixel(pos, Vector3(1.0,1.0,1.0));
        }
    }
};

struct DrawLine : public PrimitiveDraw {
    Vector3 color;
    Float stair;
    Line2 line;

    DrawLine() {
        Vector2 a{5.0, 2.0}, b{600.0, 300.0};
        line = Line2(a,b);
        color = Vector3(0.3,0.2,0.7);
        stair = 40.0;
    }

    void frag(Screen &screen, Vector2 pos) override {
        const Float th = abs(line.distance(pos));
        const Float nth = ((th - fmod(th, stair))/ 700.0);
        if (th <= 1.0) {
            screen.setPixel(pos, Vector3(1.0,1.0,1.0));
        } else {
            screen.setPixel(pos, (1.0-nth) * color);
        }
    }
};

struct DrawTriangle : public PrimitiveDraw {
    Vector3 color;
    Float stair;
    Triangle tri;

    DrawTriangle() {
        Vector2 a{5.0, 2.0}, b{600.0, 300.0}, c{200, 200};
        tri = Triangle(a,b,c);
        color = Vector3(0.3,0.2,0.7);
        stair = 40.0;
    }

    void frag(Screen &screen, Vector2 pos) override {
        Vector3 bc = tri(pos);
        if (tri.test(pos)) {
            screen.setPixel(pos, bc);
        } else {
            screen.setPixel(pos, Vector3(1.0,1.0,1.0));
        }
    }
};

int main() {
    ObjLoad app;
    Screen screen(WIDTH, HEIGHT);
    if (!glfwInit())
         return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    
    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "FOCG Demo", NULL, NULL);
    if (!window) {
        return 1;
    }

    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
    glfwSwapInterval(1);
    
    KeyboardState key{window};
    
    while (!glfwWindowShouldClose(window)) {
        app.render(screen, key);
        glRasterPos2f(-1,-1);
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glPixelZoom((double)width/WIDTH, (double)height/HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawPixels(WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, screen.data());
        glfwSwapBuffers(window);
        glfwPollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    glfwDestroyWindow(window);
    
    return 0;
}
