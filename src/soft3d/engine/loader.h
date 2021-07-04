#pragma once
#include <soft3d/common/linalg.h>
#include <soft3d/common/texture.h>

#include <fstream>
#include <vector>

struct ModelTriangle {
    Vector3 a;
    Vector3 b;
    Vector3 c;
    Vector3 nA;
    Vector3 nB;
    Vector3 nC;
    Vector2 tA;
    Vector2 tB;
    Vector2 tC;
};

struct Mesh {
    std::vector<ModelTriangle> data;
};

struct Model {
    std::vector<Mesh> meshes;
};

Model loadObj(std::string path);
Image loadTexture(std::string path);
