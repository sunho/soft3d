#pragma once
#include <soft3d/image/texture.h>
#include <soft3d/math/linalg.h>
#include <soft3d/scene/geom.h>

#include <fstream>
#include <vector>

struct Mesh {
    std::vector<TriangleVertex> vertices;

    std::vector<Triangle> generateTriangles(Material material, TextureId tex);
};

struct Model {
    std::vector<Mesh> meshes;
};

Model loadObj(std::string path);
Image loadTexture(std::string path);
