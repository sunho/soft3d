#include <focg/common/curve.h>
#include <focg/common/screen.h>
#include <focg/common/loader.h>
#include <focg/common/scene.h>
#include <focg/common/transform.h>
#include <focg/backend/cpu/renderer.h>
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

struct ObjLoad {
    Model model;
    std::unique_ptr<Renderer> renderer;

    ObjLoad() : renderer(std::make_unique<CPURenderer>()){
        Scene scene;
        model = loadObj("model.obj");
        
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(-0.5, 0.0, 2.0), basis, 2.0);
        StdLightSystem lightSystem;
        lightSystem.ambientIntensity = 0.1;
        lightSystem.lights.push_back({1.2, Vector3(-0.5, 0.5, 0.5).normalized()});
        lightSystem.lights.push_back({0.3, Vector3(-0.8, -1.0, 0.5).normalized()});
        scene.lightSystem = lightSystem;
        
        Shade shade1 = {
            .diffuse = Vector3(0.5,1.0,0.5),
            .ambient = Vector3(0.5,1.0,0.5),
            .specular = Vector3(0.3, 0.3, 0.3),
            .reflect = Vector3(0.2,0.2,0.2),
            .phong = 100.0
        };
    
        Shade shade2 = {
            .diffuse = Vector3(0.5,0.5,1.0),
            .ambient = Vector3(0.5,0.5,1.0),
            .specular = Vector3(0.3, 0.3, 0.3),
            .reflect = Vector3(0.2,0.2,0.2),
            .phong = 100.0
        };
        Shade shade3 = {
            .diffuse = Vector3(0.7,0.7,0.7),
            .ambient = Vector3(0.7,0.7,0.7),
            .specular = Vector3(0.3, 0.3, 0.3),
            .reflect = Vector3(0.2,0.2,0.2),
            .phong = 100.0
        };
        
        Float sz = -0.3;
        scene.geoms.push_back(Triangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0,  sz), Vector3(2.0, 2.0,  sz), shade3));
        scene.geoms.push_back(Triangle(Vector3(-1.0, -1.0,  sz), Vector3(1.0, -2.0, sz), Vector3(2.0, 2.0,  sz),  shade3));
        scene.geoms.push_back(Sphere(Vector3(0.0, 0.0, -0.15), 0.10, shade2));

        for(auto tri : model.meshes[0].data) {
            scene.geoms.push_back(Triangle(tri.a, tri.b, tri.c,  shade1));
        }
        renderer->sceneRef() = scene;
    }
    
    void render(Screen &screen, KeyboardState& key) {
        renderer->render(screen);
    }
};

struct SphereRayTrace {
    std::unique_ptr<Renderer> renderer;
    
    SphereRayTrace()  : renderer(std::make_unique<CPURenderer>()) {
        Scene scene;
        Shade shade1 = {
            .diffuse = Vector3(0.5,1.0,0.5),
            .ambient = Vector3(0.5,1.0,0.5),
            .specular = Vector3(0.3, 0.3, 0.3),
            .phong = 100.0
        };
        Shade shade2 = {
            .diffuse = Vector3(0.5,0.5,1.0),
            .ambient = Vector3(0.5,0.5,1.0),
            .specular = Vector3(0.3, 0.3, 0.3),
            .phong = 100.0
        };
        Shade shade3 = {
            .diffuse = Vector3(0.5,0.5,0.5),
            .ambient = Vector3(0.5,0.5,0.5),
            .specular = Vector3(0.3, 0.3, 0.3),
            .reflect = Vector3(0.2,0.2,0.2),
            .phong = 100.0
        };
        Shade shade4 = {
            .diffuse = Vector3(0.8,0.8,0.8),
            .ambient = Vector3(0.8,0.8,0.8),
            .specular = Vector3(0.3, 0.3, 0.3),
            .reflect = Vector3(0.2,0.2,0.2),
            .phong = 100.0
        };
        StdLightSystem lightSystem;
        lightSystem.ambientIntensity = 0.2;
        lightSystem.lights.push_back({1.2, Vector3(-0.5, 0.5, 0.5).normalized()});
        lightSystem.lights.push_back({0.3, Vector3(-0.8, -1.0, 0.5).normalized()});
        lightSystem.lights.push_back({0.3, Vector3(0.8, -1.0, 0.5).normalized()});
        scene.lightSystem = lightSystem;
        
        scene.geoms.push_back(Sphere(Vector3(-0.2, 0.0, 0.0), 0.25, shade1));
        scene.geoms.push_back(Sphere(Vector3(0.3, 0.0, -0.4), 0.25, shade2));
        scene.geoms.push_back(Triangle(Vector3(-0.5, 1.0, -0.2), Vector3(-0.5, -0.5,  -0.1), Vector3(0.5, 0.5,  -0.5), shade4));
        Float sz = -0.6;
        scene.geoms.push_back(Triangle(Vector3(-1.0, 2.0, sz), Vector3(-1.0, -1.0,  sz), Vector3(2.0, 2.0,  sz),  shade3));
        scene.geoms.push_back(Triangle(Vector3(-1.0, -1.0,  sz), Vector3(1.0, -2.0, sz), Vector3(2.0, 2.0,  sz),  shade3));
        Sphere& sp = std::get<Sphere>(scene.geoms[0]);
        sp.transform = toHomo(scale3(1.0,0.5,1.0));
        sp.itransform = *invertMatrix4x4(sp.transform);
        
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, 1.0);
        scene.camera = Camera(Vector3(-0.5, -0.5, 2.0), basis, 2.0);
        renderer->sceneRef() = scene;
    }
    
    void render(Screen &screen, KeyboardState& key) {
        Camera& camera = renderer->sceneRef().camera;
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
        renderer->render(screen);
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
    Triangle2 tri;

    DrawTriangle() {
        Vector2 a{5.0, 2.0}, b{600.0, 300.0}, c{200, 200};
        tri = Triangle2(a,b,c);
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

struct DrawTransformedTriangle : public PrimitiveDraw {
    Vector3 color;
    Float stair;
    Triangle2 tri;
    Matrix transform;
    Float deg = 0;

    DrawTransformedTriangle() {
        color = Vector3(0.3,0.2,0.7);
        stair = 40.0;
    }
    
    void begin(Screen &screen, KeyboardState& key) override {
        deg += 5.0;
        Vector2 a{5.0, 2.0}, b{600.0, 300.0}, c{200, 200};
        transform = scale2(0.6, 0.6) * shearY2(0.5) * shearX2(-0.5) * rotate2(deg2rad(deg));
        
        a = transform.mul<Vector2>(a);
        b = transform.mul<Vector2>(b);
        c = transform.mul<Vector2>(c);
        tri = Triangle2(a,b,c);
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
    SphereRayTrace app;
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
