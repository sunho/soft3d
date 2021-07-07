#include "loader.h"

#include <stb_image.h>

std::vector<Triangle> Model::generateTriangles(int mesh, Material material, TextureId tex) {
    Mesh& mesh_ = meshes[mesh];
    std::vector<Triangle> out;
    for (int i = 0; i < mesh_.indices.size(); i += 3) {
        TriangleVertex a{ v[mesh_.indices[i].v], vn[mesh_.indices[i].vn], vt[mesh_.indices[i].vt] };
        TriangleVertex b{ v[mesh_.indices[i + 1].v], vn[mesh_.indices[i + 1].vn],
                          vt[mesh_.indices[i + 1].vt] };
        TriangleVertex c{ v[mesh_.indices[i + 2].v], vn[mesh_.indices[i + 2].vn],
                          vt[mesh_.indices[i + 2].vt] };
        Triangle tri(a, b, c, material);
        tri.texture = tex;
        out.push_back(tri);
    }
    return out;
}

std::vector<std::string> split(std::string input, char delimiter) {
    std::vector<std::string> answer;
    std::stringstream ss(input);
    std::string temp;

    while (getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }

    return answer;
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
    std::vector<MeshIndex> indices;
    std::string line;
    const auto flush = [&]() {
        if (vertices.size() == 0) {
            return;
        }
        model.meshes.push_back(Mesh{ indices });
        indices.clear();
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
            } else if (command == "f") {
                std::vector<std::string> vets = split(line.substr(pos), ' ');
                std::vector<int> cords;
                std::for_each(vets.begin(), vets.end(), [&](std::string str) {
                    std::vector<std::string> cordStrs = split(str, '/');
                    for (auto& cordStr : cordStrs) {
                        cords.push_back(std::stoi(cordStr));
                    }
                });
                if (cords.size() != 9) {
                    throw std::runtime_error("Invalid index");
                }
                indices.push_back({ cords[0] - 1, cords[1] - 1, cords[2] - 1 });
                indices.push_back({ cords[3] - 1, cords[4] - 1, cords[5] - 1 });
                indices.push_back({ cords[6] - 1, cords[7] - 1, cords[8] - 1 });
            } else {
                printf("unknwon: %s\n", line.c_str());
            }
        }
    }
    flush();
    model.v = vertices;
    model.vn = normals;
    model.vt = texs;
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
