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
