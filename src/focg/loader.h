#pragma once
#include <fstream>
#include <vector>
#include <focg/geom.h>

struct ModelTriangle {
    Vector3 a;
    Vector3 b;
    Vector3 c;
};

struct Mesh {
    std::vector<ModelTriangle> data;
};

struct Model {
    std::vector<Mesh> meshes;
};

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
    std::string line;
    const auto flush = [&]() {
        if (vertices.size() == 0) {
            return;
        }
        model.meshes.push_back(Mesh{});
        Mesh& mesh = model.meshes[model.meshes.size()-1];
        for (int i = 0; i < vertices.size(); i+=3) {
            mesh.data.push_back(ModelTriangle{vertices[i], vertices[i+1], vertices[i+2]});
        }
    };
    bool process = false;
    while(getline(input, line)) {
        auto pos = line.find(" ");
        if (pos != std::string::npos) {
            std::string command = line.substr(0, pos);
            if (command == "o") {
                flush();
            } else if (command == "v") {
                double a,b,c;
                std::string s = line.substr(pos);
                sscanf(s.c_str(), "%lf %lf %lf", &a, &b, &c);
                vertices.push_back(Vector3(a,b,c));
            }
        }
    }
    flush();
    return model;
}
