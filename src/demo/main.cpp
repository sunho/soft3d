#include <focg/curve.h>
#include <focg/screen.h>
#include <focg/raytracer.h>
#include <chrono>
#include <thread>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

constexpr size_t WIDTH = 640;
constexpr size_t HEIGHT = 480;


struct PrimitiveDraw {
    void render(Screen &screen) {
        begin(screen);
        for (int i = 0; i < WIDTH; ++i) {
           for (int j = 0; j < HEIGHT; ++j) {
               Vector2 pos(i,j);
               frag(screen, pos);
           }
        }
        end(screen);
    }

    virtual void begin(Screen &screen) { }
    virtual void end(Screen &screen) { }
    virtual void frag(Screen& screen, Vector2 pos) = 0;
};

struct DrawSphere : public PrimitiveDraw {
    Sphere sphere;
    Camera camera;
    Vector3 color;
    Vector3 specular;
    Vector3 light;
    
    DrawSphere() {
        sphere = Sphere(Vector3(0.0, 0.0, 0.0), 0.25);
        Basis basis;
        basis.u = Vector3(1.0, 0.0, 0.0);
        basis.v = Vector3(0.0, 1.0, 0.0);
        basis.w = Vector3(0.0, 0.0, -1.0);
        color = Vector3(0.2,1.0,0.3);
        specular = Vector3(0.5, 0.5, 0.5);
        camera = Camera(Vector3(0.5, 0.5, -1.0), basis);
        light = Vector3(0.7, 0.3, -1.0);
        light.normalize();
    }
    
    void begin(Screen &screen) override {
        light.x() += 0.05;
        light.normalize();
    }
    
    void frag(Screen &screen, Vector2 pos) override {
        const Ray ray = camera.generateRay(pos, screen);
        Vector3 normal;
        const bool test = testRay(ray, sphere, normal);
        if (test) {
            Vector3 h = ray.dir + light;
            h.normalize();
            
            Float x = std::max(0.0, light.dot(normal));
            Float x2 = std::max(0.0, h.dot(normal));
            screen.setPixel(pos, x * color + pow(x2, 100) * specular);
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
    DrawSphere app;
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
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc) glfwGetProcAddress);
    glfwSwapInterval(1);
    
    while (!glfwWindowShouldClose(window)) {
        app.render(screen);
        glRasterPos2f(-1,-1);
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glPixelZoom((double)width/WIDTH, (double)height/HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawPixels(WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, screen.data());
        glfwSwapBuffers(window);
        glfwPollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    glfwDestroyWindow(window);
    
    return 0;
}
