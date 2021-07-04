#include "loader.h"

#include <stb_image.h>

// This is slow
Model loadObj(std::string path) {
    std::ifstream input;
    input.open(path);
    if (!input) {
        printf("DIE\n");
        std::terminate();
    }
    Model model;
    std::vector<Vector3> vertices;
    std::vector<Vector3> normals;
    std::string line;
    const auto flush = [&]() {
        if (vertices.size() == 0) {
            return;
        }
        model.meshes.push_back(Mesh{});
        Mesh& mesh = model.meshes[model.meshes.size() - 1];
        for (int i = 0; i < vertices.size(); i += 3) {
            mesh.data.push_back(ModelTriangle{ vertices[i], vertices[i + 1], vertices[i + 2],
                                               normals[i], normals[i + 1], normals[i + 2] });
        }
    };
    bool process = false;
    while (getline(input, line)) {
        auto pos = line.find(" ");
        if (pos != std::string::npos) {
            std::string command = line.substr(0, pos);
            if (command == "o") {
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
        printf("DIE\n");
        std::terminate();
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
