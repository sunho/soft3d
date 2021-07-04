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

constexpr size_t WIDTH = 500;
constexpr size_t HEIGHT = 500;

struct KeyboardState {
    GLFWwindow* window;
    bool pressed(int keycode) {
        int state = glfwGetKey(window, keycode);
        return (state == GLFW_PRESS);
    }
};

struct Demo2D {
    virtual ~Demo2D() {
    }
    virtual void render(Image& screen, KeyboardState& key) = 0;
};

struct PrimitiveDraw : public Demo2D {
    void render(Image& screen, KeyboardState& key) override {
        begin(screen, key);
        for (int i = 0; i < WIDTH; ++i) {
            for (int j = 0; j < HEIGHT; ++j) {
                Vector2 pos(i, j);
                frag(screen, pos);
            }
        }
        end(screen, key);
    }

    virtual void begin(Image& screen, KeyboardState& key) {
    }
    virtual void end(Image& screen, KeyboardState& key) {
    }
    virtual void frag(Image& screen, Vector2 pos) = 0;
};

struct DrawLine : public PrimitiveDraw {
    Vector3 color;
    Float stair;
    Line2 line;

    DrawLine() {
        Vector2 a{ 5.0, 2.0 }, b{ 600.0, 300.0 };
        line = Line2(a, b);
        color = Vector3(0.3, 0.2, 0.7);
        stair = 40.0;
    }
    ~DrawLine() {
    }

    void frag(Image& screen, Vector2 pos) override {
        const Float th = abs(line.distance(pos));
        const Float nth = ((th - fmod(th, stair)) / 700.0);
        if (th <= 1.0) {
            screen.setPixel(pos, Vector3(1.0, 1.0, 1.0));
        } else {
            screen.setPixel(pos, (1.0 - nth) * color);
        }
    }
};

struct DrawTriangle : public PrimitiveDraw {
    Vector3 color;
    Float stair;
    Triangle2 tri;

    DrawTriangle() {
        Vector2 a{ 5.0, 2.0 }, b{ 600.0, 300.0 }, c{ 200, 200 };
        tri = Triangle2(a, b, c);
        color = Vector3(0.3, 0.2, 0.7);
        stair = 40.0;
    }
    ~DrawTriangle() {
    }

    void frag(Image& screen, Vector2 pos) override {
        Vector3 bc = tri(pos);
        if (tri.test(pos)) {
            screen.setPixel(pos, bc);
        } else {
            screen.setPixel(pos, Vector3(1.0, 1.0, 1.0));
        }
    }
};

struct DrawTriangleZ : public Demo2D {
    Vector3 color;
    Vector3 color2;
    Vector3 color3;
    Triangle2 tri;

    DrawTriangleZ() {
        Vector2 a{ 5.0, 2.0 }, b{ 500.0, 400.0 }, c{ 200, 200 };
        tri = Triangle2(a, b, c);
        color = Vector3(0.3, 0.2, 0.7);
        color2 = Vector3(0.9, 0.3, 0.7);
        color3 = Vector3(0.3, 0.7, 0.2);
    }
    ~DrawTriangleZ() {
    }

    void render(Image& screen, KeyboardState& key) override {
        tri.draw(screen, color, color2, color3);
    }
};

struct DrawTransformedTriangle : public PrimitiveDraw {
    Vector3 color;
    Float stair;
    Triangle2 tri;
    Matrix transform;
    Float deg = 0;

    DrawTransformedTriangle() {
        color = Vector3(0.3, 0.2, 0.7);
        stair = 40.0;
    }
    ~DrawTransformedTriangle() {
    }

    void begin(Image& screen, KeyboardState& key) override {
        deg += 5.0;
        Vector2 a{ 5.0, 2.0 }, b{ 600.0, 300.0 }, c{ 200, 200 };
        transform = scale2(0.6, 0.6) * shearY2(0.5) * shearX2(-0.5) * rotate2(deg2rad(deg));

        a = transform.mul<Vector2>(a);
        b = transform.mul<Vector2>(b);
        c = transform.mul<Vector2>(c);
        tri = Triangle2(a, b, c);
    }

    void frag(Image& screen, Vector2 pos) override {
        Vector3 bc = tri(pos);
        if (tri.test(pos)) {
            screen.setPixel(pos, bc);
        } else {
            screen.setPixel(pos, Vector3(1.0, 1.0, 1.0));
        }
    }
};

struct DrawLineZ : public Demo2D {
    Vector3 color;
    Float stair;
    Line2 line;

    DrawLineZ() {
        Vector2 a{ 5.0, 200.0 }, b{ 50.0, 150.0 };
        line = Line2(a, b);
        printf("%f\n", line.slope);
        color = Vector3(0.3, 0.2, 0.7);
    }
    ~DrawLineZ() {
    }

    void render(Image& screen, KeyboardState& key) override {
        line.draw(screen, color);
    }
};

struct FilterSeq : public Demo2D {
    Sequence1 seq;
    Sequence1 filtered;

    FilterSeq() {
        prepare();
    }
    ~FilterSeq() {
    }

    void prepare() {
        const int N = 500;
        seq = Sequence1(N);
        filtered = Sequence1(N);
        for (int i = 0; i < N; ++i) {
            seq[i] = (rand() % 200) / 200.0;
        }
        ContFilter1 filter(2.0, [](Float x) {
            if (-1.1 <= x && x <= 1.1) {
                auto out = (-3 * pow(1.0 - fabs(x), 3.0) + 3 * pow(1.0 - fabs(x), 2.0) +
                            3 * (1.0 - fabs(x)) + 1) /
                           6.0;
                return out;
            } else {
                return (pow((2 - fabs(x)), 3.0)) / 6.0;
            }
        });
        int s = 3.0;
        ContFilter1 filter2(2.0 * s, [=](Float x) { return filter(x / s) / s; });

        for (int i = 0; i < N; ++i) {
            filtered[i] = reconstruct(seq, filter2, i);
        }
    }

    void render(Image& screen, KeyboardState& key) override {
        screen.clear();
        prepare();
        const int columnSize = screen.getWidth() / seq.getSize();

        for (int i = 0; i < seq.getSize(); ++i) {
            const int x0 = i * columnSize + (columnSize / 2);
            const int x1 = i * columnSize;
            const int x2 = (i + 1) * columnSize;
            const Vector2 p0 = Vector2(x0, 200 * filtered[i]);
            const Vector2 p1 = Vector2(x1, 0);
            const Vector2 p2 = Vector2(x2, 0);
            Triangle2 tri(p0, p1, p2);
            tri.draw(screen, Vector3(0x92daf0), Vector3(1.0, 1.0, 1.0), Vector3(1.0, 1.0, 1.0));
        }

        for (int i = 0; i < seq.getSize(); ++i) {
            const int x0 = i * columnSize + (columnSize / 2);
            const int x1 = i * columnSize;
            const int x2 = (i + 1) * columnSize;
            const Vector2 p0 = Vector2(x0, 200 + 200 * seq[i]);
            const Vector2 p1 = Vector2(x1, 200);
            const Vector2 p2 = Vector2(x2, 200);
            Triangle2 tri(p0, p1, p2);
            tri.draw(screen, Vector3(0xf59b5b), Vector3(1.0, 1.0, 1.0), Vector3(0xf59b5b));
        }
    }
};

int runDemo2D(Demo2D* demo) {
    Image screen(WIDTH, HEIGHT);
    if (!glfwInit())
        return 1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    GLFWwindow* window = glfwCreateWindow(WIDTH, HEIGHT, "soft3d Demo", NULL, NULL);
    if (!window) {
        return 1;
    }

    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(1);

    KeyboardState key{ window };

    while (!glfwWindowShouldClose(window)) {
        demo->render(screen, key);
        screen.pack();
        glRasterPos2f(-1, -1);
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        glViewport(0, 0, width, height);
        glPixelZoom((double)width / WIDTH, (double)height / HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT);
        glDrawPixels(WIDTH, HEIGHT, GL_RGBA, GL_UNSIGNED_BYTE, screen.data());
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    return 0;
}

#define RUN2D(demo)                                                                                \
    {                                                                                              \
        Demo2D* d = new demo;                                                                      \
        runDemo2D(d);                                                                              \
        delete d;                                                                                  \
    }
