#include "loader.h"

#include <stb_image.h>

std::vector<Triangle> Mesh::generateTriangles(Material material, TextureId tex) {
    std::vector<Triangle> out;
    for (int i = 0; i < vertices.size(); i += 3) {
        Triangle tri(vertices[i], vertices[i + 1], vertices[i + 2], material);
        tri.texture = tex;
        out.push_back(tri);
    }
    return out;
}

Model loadObj(std::string path) {
    std::ifstream input;
    input.open(path);
    if (!input) {
        throw std::runtime_error("Invalid path");
    }
    Model model;
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::vector<Vector2> texs;
    std::string line;
    const auto flush = [&]() {
        if (vertices.size() == 0) {
            return;
        }
        model.meshes.push_back(Mesh{});
        Mesh& mesh = model.meshes[model.meshes.size() - 1];
        for (int i = 0; i < vertices.size(); ++i) {
            if (texs.size() != 0) {
                mesh.vertices.push_back(TriangleVertex{ vertices[i], normals[i], texs[i] });
            } else {
                mesh.vertices.push_back(TriangleVertex{ vertices[i], normals[i] });
            }
        }
        vertices.clear();
        normals.clear();
        texs.clear();
    };
    bool process = false;
    while (getline(input, line)) {
        auto pos = line.find(" ");
        if (pos != std::string::npos) {
            std::string command = line.substr(0, pos);
            if (command == "o" || line.rfind("# object", 0) == 0) {
                flush();
            } else if (command == "v") {
                double a, b, c;
                std::string s = line.substr(pos);
                sscanf(s.c_str(), "%lf %lf %lf", &a, &b, &c);
                vertices.push_back(Vector3(a, b, c));
            } else if (command == "vn") {
                double a, b, c;
                std::string s = line.substr(pos);
                sscanf(s.c_str(), "%lf %lf %lf", &a, &b, &c);
                normals.push_back(Vector3(a, b, c));
            } else if (command == "vt") {
                double a, b, c;
                std::string s = line.substr(pos);
                sscanf(s.c_str(), "%lf %lf", &a, &b);
                texs.push_back(Vector2(a, b));
            }
        }
    }
    flush();
    return model;
}

Image loadTexture(std::string path) {
    int width, height, cn;
    stbi_set_flip_vertically_on_load(1);
    auto* img = stbi_load(path.c_str(), &width, &height, &cn, 0);
    uint32_t* buffer = reinterpret_cast<uint32_t*>(img);
    if (!img) {
        throw std::runtime_error("Invalid path");
    }
    Image out(width, height);
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            Vector3 color =
                out.unpackPixel(*reinterpret_cast<uint32_t*>(&img[j * width * cn + i * cn]));
            out.setPixel(i, j, color);
        }
    }
    return out;
}
