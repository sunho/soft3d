#include "runtime.h"

// clang-format off
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// clang-format on

#include <soft3d/renderer/rtcpu/rtcpu.h>
#include <soft3d/renderer/zcpu/zcpu.h>

#include <stb_image_write.h>

#include <chrono>
#include <thread>

struct GLFWWindowImpl : public WindowImpl {
    GLFWWindowImpl() {
    }

    ~GLFWWindowImpl() {
        if (window) {
            glfwDestroyWindow(window);
        }
    }

    void createWindow(int& width, int& height) override {
        if (!glfwInit())
            return;

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

        window = glfwCreateWindow(width, height, "FOCG", NULL, NULL);
        if (!window) {
            return;
        }
        glfwGetFramebufferSize(window, &width, &height);
        windowWidth = width;
        windowHeight = height;

        glfwMakeContextCurrent(window);
        gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
        glfwSwapInterval(1);
    }

    bool pressed(int keycode) override {
        int state = glfwGetKey(window, keycode);
        return (state == GLFW_PRESS);
    }

    void setTitle(std::string title) override {
        glfwSetWindowTitle(window, title.c_str());
    }

    void runWindowLoop() override {
        while (!glfwWindowShouldClose(window)) {
            uint32_t* buffer = updateFunc();
            glClear(GL_COLOR_BUFFER_BIT);
            glDrawPixels(windowWidth, windowHeight, GL_RGBA, GL_UNSIGNED_BYTE, buffer);
            glfwSwapBuffers(window);
            glfwPollEvents();
        }
    }

    void setUpdateFunc(UpdateFunc func) override {
        updateFunc = func;
    }

  private:
    size_t windowWidth;
    size_t windowHeight;
    GLFWwindow* window;
    UpdateFunc updateFunc;
};

Runtime::Runtime(RuntimeConfig conf) : conf(conf), window(std::make_unique<GLFWWindowImpl>()) {
    lastFps.fill(0.0);
    initRenderer();
}

void Runtime::initRenderer() {
    switch (conf.renderer) {
        case Backend::RTCPU:
            renderer = std::make_unique<RTCPURenderer>(RTCPUConfig());
            break;
        case Backend::ZCPU:
            renderer = std::make_unique<ZCPURenderer>(ZCPUConfig());
            break;
    }
}

void Runtime::run(App* app) {
    this->app = app;
    app->init(*this, renderer->sceneRef());
    window->setUpdateFunc([=]() { return this->update(); });
    window->createWindow(conf.width, conf.height);
    screen = Image(conf.width, conf.height);
    lastFrame = std::chrono::system_clock::now();
    window->runWindowLoop();
}

void Runtime::render(App* app, std::string path) {
    this->app = app;
    app->init(*this, renderer->sceneRef());
    screen = Image(conf.width, conf.height);
    lastFrame = std::chrono::system_clock::now();
    uint32_t* data = update();
    stbi_flip_vertically_on_write(1);
    stbi_write_png(path.c_str(), conf.width, conf.height, 4, data, conf.width * 4);
}

uint32_t* Runtime::update() {
    const auto currentFrame = std::chrono::system_clock::now();
    const int dt =
        std::chrono::duration_cast<std::chrono::milliseconds>(currentFrame - lastFrame).count();
    app->update(*this, renderer->sceneRef(), dt);
    renderer->render(screen);
    Image newImage;
    if (conf.aaProfile) {
        newImage = filterImage(screen, conf.aaProfile->filter);
        newImage.pack();
    } else {
        screen.pack();
    }
    lastFrame = currentFrame;

    const auto diff = std::chrono::system_clock::now() - currentFrame;
    const int elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(diff).count();
    const double fps = 1000.0 / dt;
    for (int i = 1; i < lastFps.size(); ++i) {
        lastFps[i] = lastFps[i - 1];
    }
    lastFps[0] = fps;

    if (fps > conf.fps) {
        const double desiredMs = 1000.0 / conf.fps;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(desiredMs - elapsedTime) + 1));
    }
    window->setTitle(std::string("FOCG (frame time: ") + std::to_string((int)elapsedTime) +
                     "ms, fps: " + std::to_string((int)this->fps()) + ")");

    return conf.aaProfile ? newImage.data() : screen.data();
}

bool Runtime::pressed(int keycode) {
    return window->pressed(keycode);
}

double Runtime::fps() {
    double out = 0;
    for (auto fps : lastFps)
        out += fps;
    out /= lastFps.size();
    return out;
}
