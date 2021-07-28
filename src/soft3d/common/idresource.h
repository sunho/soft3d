#pragma once

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
        available.push_back(id);
        registered.erase(std::find(registered.begin(), registered.end(), id));
        resources.erase(id);
    }

    const std::vector<int>& ids() {
        return registered;
    }

    inline T* get(int id) {
        return table[id - 1];
    }

    T* draw() {
        size_t i = rand() % registered.size();
        return table[registered[i]-1];
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
            registered.push_back(nextId);
            return nextId;
        }
        const int out = *available.end();
        registered.push_back(out);
        available.erase(available.end());
        return out;
    }

    std::vector<T*> table;
    int nextId{ 0 };
    std::map<int, T> resources;

    std::vector<int> available;
    std::vector<int> registered;
};
