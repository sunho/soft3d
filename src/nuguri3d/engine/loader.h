#pragma once
#include <nuguri3d/common/linalg.h>
#include <nuguri3d/common/texture.h>

#include <fstream>
#include <vector>

struct ModelTriangle {
    Vector3 a;
    Vector3 b;
    Vector3 c;
    Vector3 nA;
    Vector3 nB;
    Vector3 nC;
};

struct Mesh {
    std::vector<ModelTriangle> data;
};

struct Model {
    std::vector<Mesh> meshes;
};

Model loadObj(std::string path);
Image loadTexture(std::string path);
