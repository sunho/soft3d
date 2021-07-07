#pragma once

#include <soft3d/scene/geom.h>

#include <list>
#include <map>
#include <set>
#include <variant>

struct BvhNode {
    BvhNode() = default;
    explicit BvhNode(BoundingRect rect) : rect(rect) {
    }
    virtual ~BvhNode() {
    }
    virtual bool isBox() = 0;
    BoundingRect rect;
};

struct BvhGeom : public BvhNode {
    BvhGeom() = default;
    explicit BvhGeom(BoundingRect rect, Geometry geom) : BvhNode(rect), geom(geom) {
    }
    bool isBox() override {
        return false;
    }
    ~BvhGeom() {
    }
    Geometry geom;
};

struct BvhBox : public BvhNode {
    BvhBox() = default;
    explicit BvhBox(BoundingRect rect, BvhNode* left, BvhNode* right)
        : BvhNode(rect), left(left), right(right) {
    }
    ~BvhBox() {
    }
    bool isBox() override {
        return true;
    }

    BvhNode* left{ nullptr };
    BvhNode* right{ nullptr };
};
using LeafTestFunc =
    std::function<bool(const Geometry* geom, Ray ray, Float t0, Float t1, RayHit& hit)>;

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

    void add(const Geometry& geom) {
        leafs.push_back(std::make_pair(getBoundingRect(geom), geom));
        dirty = true;
    }

    void prepare() {
        if (dirty)
            [[unlikely]] {
                buildTree();
                dirty = false;
            }
    }

    bool testRay(Ray ray, Float t0, Float t1, RayHit& hit, const LeafTestFunc& func) {
        return testRayInternal(*head, ray, t0, t1, hit, func);
    }

  private:
    bool testRayInternal(BvhNode& node, Ray ray, Float t0, Float t1, RayHit& hit,
                         const LeafTestFunc& func) {
        if (node.rect.hit(ray)) {
            if (node.isBox()) {
                BvhBox& box = static_cast<BvhBox&>(node);
                RayHit leftHit;
                RayHit rightHit;
                const bool leftTest =
                    box.left && testRayInternal(*box.left, ray, t0, t1, leftHit, func);
                const bool rightTest =
                    box.right && testRayInternal(*box.right, ray, t0, t1, rightHit, func);
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
                BvhGeom& geom = static_cast<BvhGeom&>(node);
                return func(&geom.geom, ray, t0, t1, hit);
            }
        } else {
            return false;
        }
    }

    void buildTree() {
        nodes.clear();
        head = buildTreeInternal(0, leafs.size(), 0);
    }

    BvhNode* buildTreeInternal(int left, int right, int axis) {
        const int size = right - left;
        if (size == 1) {
            auto node = std::make_unique<BvhGeom>(leafs[left].first, leafs[left].second);

            BvhNode* out = node.get();
            nodes.push_back(std::move(node));
            return out;
        } else if (size == 2) {
            auto leftNode = std::make_unique<BvhGeom>(leafs[left].first, leafs[left].second);
            auto rightNode =
                std::make_unique<BvhGeom>(leafs[left + 1].first, leafs[left + 1].second);

            const BoundingRect rect = leftNode->rect + rightNode->rect;
            auto box = std::make_unique<BvhBox>(rect, leftNode.get(), rightNode.get());
            BvhNode* out = box.get();
            nodes.push_back(std::move(leftNode));
            nodes.push_back(std::move(rightNode));
            nodes.push_back(std::move(box));
            return out;
        } else {
            // sort according to axis
            std::sort(std::next(leafs.begin(), left), std::next(leafs.begin(), right),
                      [&](const auto& lhs, const auto& rhs) {
                          return lhs.first.minComp(axis) < rhs.first.minComp(axis);
                      });
            // naively select axis there's room for better quality here

            BvhNode* leftNode = buildTreeInternal(left, (left + right) / 2, (axis + 1) % 3);
            BvhNode* rightNode = buildTreeInternal((left + right) / 2, right, (axis + 1) % 3);
            const BoundingRect rect = leftNode->rect + rightNode->rect;
            auto box = std::make_unique<BvhBox>(rect, leftNode, rightNode);
            BvhNode* out = box.get();
            nodes.push_back(std::move(box));
            return out;
        }
    }
    bool dirty{ false };
    std::vector<std::pair<BoundingRect, Geometry>> leafs;
    std::list<std::unique_ptr<BvhNode>> nodes;
    BvhNode* head{ nullptr };
};
