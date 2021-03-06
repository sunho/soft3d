#pragma once

#include <soft3d/image/filter.h>
#include <soft3d/renderer/interface.h>

#include <chrono>
#include <optional>

enum class Backend { RTCPU, ZCPU };

struct AAProfile {
    Sequence1 filter;
};

struct RuntimeConfig {
    int width;
    int height;
    int fps;
    Backend renderer;
    std::optional<AAProfile> aaProfile;
};

struct Runtime;

struct App {
    virtual ~App() {
    }
    virtual void init(Runtime& engine, Scene& scene) = 0;
    virtual void update(Runtime& engine, Scene& scene, double dt) = 0;
};

using UpdateFunc = std::function<uint32_t*()>;

struct WindowImpl {
    virtual ~WindowImpl() {
    }
    virtual void setUpdateFunc(UpdateFunc func) = 0;
    virtual void createWindow(int& width, int& height) = 0;
    virtual void runWindowLoop() = 0;
    virtual void setTitle(std::string title) = 0;
    virtual bool pressed(int keycode) = 0;
};

constexpr size_t LAST_FPS_COUNT = 5;

struct Runtime {
    Runtime() = delete;
    Runtime(RuntimeConfig conf);

    void run(App* app);
    void render(App* app, std::string path);
    void renderHdr(App* app, std::string path);
    bool pressed(int keycode);
    double fps();

  private:
    uint32_t* update();
    void initRenderer();

    RuntimeConfig conf;
    App* app{ nullptr };
    std::unique_ptr<Renderer> renderer;
    std::unique_ptr<WindowImpl> window;
    Image screen;
    std::chrono::time_point<std::chrono::system_clock> lastFrame;
    std::array<double, LAST_FPS_COUNT> lastFps;
};
