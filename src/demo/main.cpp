#include <focg/curve.h>
#include <focg/screen.h>
#include <chrono>
#include <thread>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

constexpr size_t WIDTH = 640;
constexpr size_t HEIGHT = 480;

struct PrimitiveDraw {
    void render(Screen &screen) {
        for (int i = 0; i < WIDTH; ++i) {
           for (int j = 0; j < HEIGHT; ++j) {
               Vector2 pos(i,j);
               frag(screen, pos);
           }
        }
    }

    virtual void frag(Screen& screen, Vector2 pos) = 0;
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
        if (tri.test(pos)) {
            screen.setPixel(pos, color);
        } else {
            screen.setPixel(pos, Vector3(1.0,1.0,1.0));
        }
    }
};

int main() {
    DrawTriangle app;
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
    
    while (!glfwWindowShouldClose(window))
    {
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
