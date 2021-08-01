#pragma once

#include <vector>
#include <unordered_map>

template <typename T>
class ResourceList {
  public:
    using ResourceData = std::vector<char>;

    ResourceList() = default;
    ~ResourceList() {
        while (!data.empty()) {
            auto &resource = data.begin()->second;
            T *addr = reinterpret_cast<T *>(resource.data());
            destruct(addr);
        }
    }

    template <typename G, typename... ARGS>
    G *construct(ARGS &&... args) {
        ResourceData resource(sizeof(G));
        G *addr = reinterpret_cast<G *>(resource.data());
        new (addr) G(std::forward<ARGS>(args)...);
        data.emplace(reinterpret_cast<uintptr_t>(addr), std::move(resource));
        references.push_back((T*)addr);
        return addr;
    }
    
    void destruct(T *resource) {
        references.erase(std::find(references.begin(), references.end(), resource));
        resource->~T();
        data.erase(reinterpret_cast<uintptr_t>(resource));
    }

    const std::vector<T *> &list() {
        return references;
    }

  private:
    std::vector<T *> references;
    std::unordered_map<uintptr_t, ResourceData> data;
};
