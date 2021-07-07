#pragma once
#include <soft3d/image/texture.h>
#include <soft3d/math/linalg.h>
#include <soft3d/scene/geom.h>

#include <fstream>
#include <vector>

struct MeshIndex {
    int v;
    int vt;
    int vn;
};

struct Mesh {
    std::vector<MeshIndex> indices;
};

struct Model {
    std::vector<Mesh> meshes;
    std::vector<Vector3> v;
    std::vector<Vector3> vn;
    std::vector<Vector2> vt;

    std::vector<Triangle> generateTriangles(int mesh, Material material, TextureId tex);
};

Model loadObj(std::string path);
Image loadTexture(std::string path);
