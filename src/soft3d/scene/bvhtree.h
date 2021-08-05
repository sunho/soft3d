#pragma once

#include <soft3d/scene/geom.h>

#include <list>
#include <algorithm>
#include <map>
#include <set>
#include <variant>

struct BvhNode {
    BvhNode() = default;
    explicit BvhNode(BoundingRect rect) : rect(rect), box(true) {
    }
    explicit BvhNode(BoundingRect rect, Geometry* geom) : rect(rect), geom(geom), box(false) {
    }
    bool box;
    BoundingRect rect;
    Geometry* geom;
};

// Hierarchical bounding box
// Binary tree like test
// Prepare function is not thread safe
struct BvhTree {
    BvhTree() = default;

    auto begin() {
        return leafs.begin();
    }

    auto end() {
        return leafs.end();
    }

    void add(Geometry* geom) {
        leafs.push_back(std::make_pair(geom->boundingRect(), geom));
        dirty = true;
    }

    void prepare() {
        if (dirty)
            [[unlikely]] {
                buildTree();
                dirty = false;
            }
    }

    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit) {
        Vector3 invDir(1.0f / ray.dir.x(), 1.0f / ray.dir.y(), 1.0f / ray.dir.z());
        return testRayInternal(0, ray, invDir, t0, t1, hit);
    }

  private:
    bool testRayInternal(int offset, const Ray& ray, const Vector3& invDir, Float t0, Float t1, RayHit& hit) {
        BvhNode* node = &nodes[offset];
        if (node->rect.hit(ray, invDir)) {
            if (node->box) {
                RayHit leftHit;
                RayHit rightHit;
                const bool leftTest = testRayInternal(offset * 2 + 1, ray, invDir, t0, t1, leftHit);
                const bool rightTest = testRayInternal(offset * 2 + 2, ray, invDir, t0, t1, rightHit);
                if (leftTest && rightTest) {
                    if (leftHit.time < rightHit.time) {
                        hit = leftHit;
                    } else {
                        hit = rightHit;
                    }
                    return true;
                } else if (leftTest) {
                    hit = leftHit;
                    return true;
                } else if (rightTest) {
                    hit = rightHit;
                    return true;
                }
                return false;
            } else {
                if (ray.isShadow && node->geom->alpha) {
                    return false;
                }
                return node->geom->rayTest(ray, t0, t1, hit);
            }
        } else {
            return false;
        }
    }

    void buildTree() {
        nodes.clear();
        buildTreeInternal(0, leafs.size(), 0);
    }

    void buildTreeInternal(int left, int right, int axis, int offset = 0) {
        const int size = right - left;
        if (size == 1) {
            setNode(offset, BvhNode{ leafs[left].first, leafs[left].second });
        } else {
            // sort according to axis
            std::sort(std::next(leafs.begin(), left), std::next(leafs.begin(), right),
                      [&](const auto& lhs, const auto& rhs) {
                          return lhs.first.minComp(axis) < rhs.first.minComp(axis);
                      });
            // naively select axis there's room for better quality here

            buildTreeInternal(left, (left + right) / 2, (axis + 1) % 3, offset * 2 + 1);
            buildTreeInternal((left + right) / 2, right, (axis + 1) % 3, offset * 2 + 2);
            const BoundingRect rect = nodes[offset * 2 + 1].rect + nodes[offset * 2 + 2].rect;
            setNode(offset, BvhNode{ rect });
        }
    }

    inline void setNode(int offset, const BvhNode& node) {
        if (nodes.size() <= offset) {
            nodes.resize(offset * 2);
        }
        nodes[offset] = node;
    }

    bool dirty{ false };
    std::vector<std::pair<BoundingRect, Geometry*>> leafs;
    std::vector<BvhNode> nodes;
};
