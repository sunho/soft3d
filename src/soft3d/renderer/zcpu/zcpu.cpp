#include "zcpu.h"

constexpr int THREAD_NUM = 8;
constexpr int MAX_JOB = 2000 * 2000;

ZCPURenderer::ZCPURenderer() : threadPool(THREAD_NUM, MAX_JOB) {
}

ZCPURenderer::~ZCPURenderer() {
}

Scene& ZCPURenderer::sceneRef() {
    return scene;
}

void ZCPURenderer::render(Image& screen) {
    const Matrix VPOV = scene.camera.VPOV(screen);
    bakeShadowMap(screen);

    Shader mainPass;
    mainPass.func = [&](const Vector3& bary, const Vector3& homo, const Geometry& geom,
                        Float depth) {
        if (auto triangle = std::get_if<PlainTriangle>(&geom)) {
            return shadeSingleShadedTriangle(bary, *triangle);
        } else if (auto triangle = std::get_if<Triangle>(&geom)) {
            return shadeTriangle(bary, *triangle, homo);
        }
    };
    renderInternal(screen, VPOV, mainPass);

    for (auto [_, texId] : shadowMaps) {
        scene.textures.release(texId);
    }
    shadowMaps.clear();
    lightMvp.clear();
}

void ZCPURenderer::renderInternal(Image& screen, const Matrix& mvp, Shader& shader) {
    zBuffer.resize(screen.getWidth() * screen.getHeight());
    std::fill(zBuffer.begin(), zBuffer.end(), -1.0 / 0.0);
    width = screen.getWidth();
    for (auto& [_, geom] : scene.geoms) {
        bool tmp = false;
        if (auto sphere = std::get_if<PlainSphere>(&geom)) {
            // todo
        } else if (auto triangle = std::get_if<PlainTriangle>(&geom)) {
            Vector3 h(1.0f, 1.0f, 1.0f);
            Triangle3 projected = triangle->curve.transformed(mvp, h);
            drawTriangle(screen, projected, *triangle, h, shader);
        } else if (auto triangle = std::get_if<Triangle>(&geom)) {
            Vector3 h(1.0f, 1.0f, 1.0f);
            Triangle3 projected = triangle->curve.transformed(mvp, h);
            drawTriangle(screen, projected, *triangle, h, shader);
        }
    }
}

constexpr Float far = -1.0;

constexpr Float near = 1.0;

void ZCPURenderer::bakeShadowMap(Image& screen) {
    for (auto lid : scene.lightSystem.lights.ids()) {
        Light* l = scene.lightSystem.lights.get(lid);
        if (auto light = std::get_if<DirectionalLight>(l)) {
            int texId = scene.textures.create(1000, 1000);
            Basis basis;
            basis.w = light->v;
            basis.u = Vector3(0, 0, 1).cross(basis.w).normalized();
            basis.v = basis.w.cross(basis.u);
            Matrix mvp = viewportMatrix(1000, 1000) *
                         orthProjectionMatrix(-2.0, 2.0, -2.0, 2.0, near, far) *
                         viewMatrix(basis, Vector3(0, 0, 0));
            lightMvp.emplace(lid, mvp);
            shadowMaps.emplace(lid, texId);
            Shader shadowPass;
            shadowPass.func = [&](const Vector3& bary, const Vector3& homo, const Geometry& geom,
                                  Float depth) {
                return Vector3(0.0f, 0.0f, -(depth - near) / (near - far));
            };
            renderInternal(*scene.textures.get(texId), mvp, shadowPass);
        }
    }
}

Float ZCPURenderer::getDepth(int i, int j) {
    return zBuffer[j * width + i];
}

void ZCPURenderer::setDepth(int i, int j, Float depth) {
    zBuffer[j * width + i] = depth;
}

void ZCPURenderer::drawTriangle(Image& screen, const Triangle3& triangle, const Geometry& geom,
                                const Vector3& homo, Shader& shader) {
    Triangle2 tri(Vector2(triangle.pA[0], triangle.pA[1]), Vector2(triangle.pB[0], triangle.pB[1]),
                  Vector2(triangle.pC[0], triangle.pC[1]));
    const int x0 = std::max(std::min({ triangle.pA[0], triangle.pB[0], triangle.pC[0] }), 0.0f);
    const int x1 = std::min(std::max({ triangle.pA[0], triangle.pB[0], triangle.pC[0] }) + 1,
                            (float)screen.getWidth());
    const int y0 = std::max(std::min({ triangle.pA[1], triangle.pB[1], triangle.pC[1] }), 0.0f);
    const int y1 = std::min(std::max({ triangle.pA[1], triangle.pB[1], triangle.pC[1] }) + 1,
                            (float)screen.getHeight());

    threadPool.setJobFunc(
        [x0, x1, homo, &geom, &screen, &shader, tri, triangle, this](std::tuple<int, int> co) {
            auto [is, ie] = co;
            for (int j = is; j < ie; ++j) {
                for (int i = x0; i < x1; ++i) {
                    Vector3 bary = tri(Vector2(i, j));
                    if (nearInRange(bary.x(), 0.0, 1.0) && nearInRange(bary.y(), 0.0, 1.0) &&
                        nearInRange(bary.z(), 0.0, 1.0)) {
                        Float depth = bary.x() * triangle.pA.z() + bary.y() * triangle.pB.z() +
                                      bary.z() * triangle.pC.z();
                        if (this->getDepth(i, j) > depth)
                            continue;
                        setDepth(i, j, depth);
                        screen.setPixel(i, j, shader.func(bary, homo, geom, depth));
                    }
                }
            }
        });

    int size = (y1 - y0) / THREAD_NUM;
    for (int i = 0; i < THREAD_NUM; ++i) {
        if (i == THREAD_NUM - 1) {
            threadPool.addJob(std::make_tuple(y0 + size * i, y1));
        } else {
            threadPool.addJob(std::make_tuple(y0 + size * i, y0 + size * (i + 1)));
        }
    }
    threadPool.flush(THREAD_NUM);
}

Vector3 ZCPURenderer::shadeSingleShadedTriangle(const Vector3& bary, const PlainTriangle& tri) {
    Vector3 pixel = scene.lightSystem.ambientIntensity * tri.shade.ambient;
    Vector3 normal = tri.normal(Vector3()).normalized();
    Vector3 hit = tri.vA * bary.x() + tri.vB * bary.y() + tri.vC * bary.z();

    for (auto& [lid, l] : scene.lightSystem.lights) {
        if (auto light = std::get_if<DirectionalLight>(&l)) {
            Float w = 1.0f;
            Vector3 shadowVec = hit.transformed(lightMvp.at(lid), w);
            Float dd = -(shadowVec.z() - near) / (near - far);

            Image* shadowMap = scene.textures.get(shadowMaps.at(lid));
            Float d = shadowMap->getPixel(shadowVec.x(), shadowVec.y()).z();
            if (fabs(d - dd) < 0.01) {
                Float x = std::max(0.0f, light->v.dot(normal));
                // Float x2 = std::max(0.0, h.dot(hit.normal));
                pixel += light->intensity * (x * tri.shade.diffuse);
            }
            // pixel = Vector3(0,0,d);
        }
    }
    return pixel;
}

Vector3 ZCPURenderer::shadeTriangle(const Vector3& bary, const Triangle& tri, const Vector3& homo) {
    Float w = 1.0f;
    Vector3 normal = tri.vA.normal * bary.x() + tri.vB.normal * bary.y() + tri.vC.normal * bary.z();

    Vector3 hit = tri.vA.pos * bary.x() + tri.vB.pos * bary.y() + tri.vC.pos * bary.z();
    Vector3 e = (scene.camera.e - hit).normalized();
    Vector3 diffuse;
    if (tri.texture) {
        // Perspective correction
        Vector2 uv = tri.vA.tex * bary.x() / homo.x() + tri.vB.tex * bary.y() / homo.y() +
                     tri.vC.tex * bary.z() / homo.z();
        uv /= (bary.x() * (1 / homo.x()) + bary.y() * (1 / homo.y()) + bary.z() * (1 / homo.z()));
        diffuse = sampleBilinear(*scene.textures.get(tri.texture), uv);
        /*
                // Normal mapping
                Vector3 deltaPos1 = tri.vB.pos - tri.vA.pos;
                Vector3 deltaPos2 = tri.vC.pos - tri.vA.pos;

                Vector2 deltaUV1 = tri.vB.tex - tri.vA.tex;
                Vector2 deltaUV2 = tri.vC.tex - tri.vA.tex;

                float r = 1.0f / (deltaUV1.x() * deltaUV2.y() - deltaUV1.y() * deltaUV2.x());
                Vector3 tangent = (deltaPos1 * deltaUV2.y() - deltaPos2 * deltaUV1.y()) * r;
                tangent.normalize();
                Vector3 bitangent = (deltaPos2 * deltaUV1.x() - deltaPos1 * deltaUV2.x()) * r;
                bitangent.normalize();

                Matrix tbn = Matrix(3, 3,
                                    { tangent[0], bitangent[0], normal[0], tangent[1], bitangent[1],
                                      normal[1], tangent[2], bitangent[2], normal[2] });
                Vector3 nn = sampleBilinear(*scene.textures.get(tri.normalMap), uv);
                nn = nn * 2.0f - Vector3(1.0f, 1.0f, 1.0f);
                normal = tbn.mul<Vector3>(nn);*/
    } else {
        diffuse = tri.shade.diffuse;
    }
    Vector3 pixel = scene.lightSystem.ambientIntensity * diffuse;

    for (auto& [lid, l] : scene.lightSystem.lights) {
        if (auto light = std::get_if<DirectionalLight>(&l)) {
            Vector3 shadowVec = hit.transformed(lightMvp.at(lid), w);
            Float dd = -(shadowVec.z() - near) / (near - far);
            Image* shadowMap = scene.textures.get(shadowMaps.at(lid));
            Float d = shadowMap->getPixel(shadowVec.x(), shadowVec.y()).z();
            if (fabs(d - dd) < 0.01) {
                Vector3 h = (e + light->v).normalized();

                Float x = std::max(0.0f, light->v.dot(normal));
                Float x2 = std::max(0.0f, h.dot(normal));
                pixel += light->intensity *
                         (x * diffuse + pow(x2, tri.shade.phong) * tri.shade.specular);
            }
            // pixel = Vector3(0,0,d);
        }
    }
    return pixel;
}
