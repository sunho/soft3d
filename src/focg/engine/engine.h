#pragma once

#include <focg/backend/interface.h>
#include <focg/common/filter.h>

#include <optional>

enum class Backend { RTCPU, ZCPU, RTGL };

struct AAProfile {
    Sequence1 filter;
};

struct EngineConfig {
    int width;
    int height;
    int fps;
    Backend renderer;
    std::optional<AAProfile> aaProfile;
};

struct Engine;

struct App {
    virtual ~App() {
    }
    virtual void init(Engine& engine, Scene& scene) = 0;
    virtual void update(Engine& engine, Scene& scene, double dt) = 0;
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

struct Engine {
    Engine() = delete;
    Engine(EngineConfig conf);

    void run(App* app);
    bool pressed(int keycode);
    double fps();

  private:
    uint32_t* update();
    void initRenderer();

    EngineConfig conf;
    App* app{ nullptr };
    std::unique_ptr<Renderer> renderer;
    std::unique_ptr<WindowImpl> window;
    Image screen;
    clock_t lastFrame{ 0 };
    std::array<double, LAST_FPS_COUNT> lastFps;
};
