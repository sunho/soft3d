#pragma once
#include <nuguri3d/common/curve.h>
#include <nuguri3d/common/geom.h>
#include <nuguri3d/common/image.h>
#include <nuguri3d/common/linalg.h>
#include <nuguri3d/common/texture.h>
#include <nuguri3d/common/transform.h>

#include <list>
#include <map>
#include <variant>
#include <vector>

struct Ray {
    Vector3 origin;
    Vector3 dir;
};

struct Camera {
    Vector3 e;
    Basis basis;
    Float focal;

    Camera() = default;
    explicit Camera(Vector3 e, Basis basis, Float focal) : e(e), basis(basis), focal(focal) {
    }

    Ray generateRay(const Vector2& pos, const Image& screen) {
        const Float u = (pos.x() + 0.5) / screen.getWidth() - 0.5;
        const Float v = (pos.y() + 0.5) / screen.getHeight() - 0.5;
        const Vector3 dir = -1 * focal * basis.w + u * basis.u + v * basis.v;
        return Ray{ e, dir.normalized() };
    }

    Matrix VPOV(const Image& screen) {
        const Matrix v = viewMatrix(basis, e);
        const Matrix p = perspectiveProjectionMatrix(-1.0, -2.0);
        const Matrix o = orthProjectionMatrix(-0.5, 0.5, -0.5, 0.5, -0.5, -2.0);
        const Matrix vp = viewportMatrix(screen.getWidth(), screen.getHeight());
        return vp * o * p * v;
    }
};

struct UniformLight {
    Float intensity;
    Vector3 v;
};

struct StdLightSystem {
    Float ambientIntensity;
    std::vector<UniformLight> lights;
};

using LightSystem = std::variant<StdLightSystem>;

struct Scene {
    Scene() = default;

    TextureId createTexture(int width, int height) {
        const TextureId id = ++lastTextureId;
        textures.emplace(id, Image(width, height));
        texturesTable.push_back(&textures.at(id));
        return id;
    }

    TextureId registerTexture(Image&& image) {
        const TextureId id = ++lastTextureId;
        textures.emplace(id, image);
        texturesTable.push_back(&textures.at(id));
        return id;
    }

    inline Image* getTexture(TextureId id) {
        return texturesTable[id - 1];
    }

    std::vector<Geometry> geoms;
    Camera camera;
    LightSystem lightSystem;

  private:
    std::vector<Image*> texturesTable;
    std::map<TextureId, Image> textures;
    TextureId lastTextureId{ 0 };
};
