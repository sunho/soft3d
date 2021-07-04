#pragma once
#include <soft3d/common/curve.h>
#include <soft3d/common/geom.h>
#include <soft3d/common/image.h>
#include <soft3d/common/linalg.h>
#include <soft3d/common/texture.h>
#include <soft3d/common/transform.h>

#include <list>
#include <map>
#include <set>
#include <variant>
#include <vector>

template <typename T>
struct IdResourceManager {
    IdResourceManager() = default;
    ~IdResourceManager() = default;
    IdResourceManager(const IdResourceManager<T>& other) {
        nextId = other.nextId;
        resources = other.resoucres;
        registered = other.registered;
        available = other.available;
        table.resize(other.table.size());
        for (auto id : registered) {
            table[id - 1] = &resources.at(id);
        }
    }

    IdResourceManager& operator=(const IdResourceManager<T>& other) {
        nextId = other.nextId;
        resources = other.resources;
        registered = other.registered;
        available = other.available;
        table.resize(other.table.size());
        for (auto id : other.registered) {
            table[id - 1] = &resources.at(id);
        }
        return *this;
    }

    IdResourceManager(IdResourceManager<T>&& other) {
        std::swap(nextId, other.nextId);
        std::swap(resources, other.resoucres);
        std::swap(registered, other.registered);
        std::swap(available, other.available);
        table.resize(other.table.size());
        for (auto id : registered) {
            table[id - 1] = &resources.at(id);
        }
    }

    IdResourceManager& operator=(const IdResourceManager<T>&& other) {
        std::swap(nextId, other.nextId);
        std::swap(resources, other.resoucres);
        std::swap(registered, other.registered);
        std::swap(available, other.available);
        table.resize(other.table.size());
        for (auto id : registered) {
            table[id - 1] = &resources.at(id);
        }
        return *this;
    }

    template <typename... Args>
    int create(Args&&... args) {
        const int id = getId();
        resources.emplace(id, T(std::forward<Args>(args)...));
        table[id - 1] = &resources.at(id);
        return id;
    }

    int move(T&& obj) {
        const int id = getId();
        resources.emplace(id, std::move(obj));
        table[id - 1] = &resources.at(id);
        return id;
    }

    void release(int id) {
        available.insert(id);
        registered.erase(id);
        resources.erase(id);
    }

    const std::set<int>& ids() {
        return registered;
    }

    inline T* get(int id) {
        return table[id - 1];
    }

    auto begin() {
        return resources.begin();
    }

    auto end() {
        return resources.end();
    }

  private:
    int getId() {
        if (available.empty()) {
            ++nextId;
            table.push_back(nullptr);
            registered.insert(nextId);
            return nextId;
        }
        const int out = *available.begin();
        registered.insert(out);
        available.erase(available.begin());
        return out;
    }

    std::vector<T*> table;
    int nextId{ 0 };
    std::map<int, T> resources;

    std::set<int> available;
    std::set<int> registered;
};

struct Ray {
    Vector3 origin;
    Vector3 dir;
};

struct Camera {
    Vector3 e;
    Basis basis;
    Float focal;

    Camera() = default;
    explicit Camera(Vector3 e, Basis basis, Float focal) : e(e), basis(basis), focal(focal) {
    }

    Ray generateRay(const Vector2& pos, const Image& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth() - 0.5;
        const Float v = (pos.y() + 0.5) / screen.getHeight() - 0.5;
        const Vector3 dir = -1 * focal * basis.w + u * basis.u + v * basis.v;
        return Ray{ e, dir.normalized() };
    }

    Matrix VPOV(const Image& screen) {
        const Matrix v = viewMatrix(basis, e);
        const Matrix p = perspectiveProjectionMatrix(-1.0, -2.0);
        const Matrix o = orthProjectionMatrix(-0.5, 0.5, -0.5, 0.5, -0.5, -2.0);
        const Matrix vp = viewportMatrix(screen.getWidth(), screen.getHeight());
        return vp * o * p * v;
    }
};

struct DirectionalLight {
    Float intensity;
    Vector3 v;
};

using Light = std::variant<DirectionalLight>;

struct LightSystem {
    LightSystem() = default;
    Float ambientIntensity;
    IdResourceManager<Light> lights;
};

struct Scene {
    Scene() = default;

    IdResourceManager<Image> textures;
    std::vector<Geometry> geoms;
    Camera camera;
    LightSystem lightSystem;
};
