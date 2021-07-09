#include "zcpu.h"

void ZBuffer::reset(int width, int height) {
    data.resize(width * height);
    this->width = width;
    this->height = height;
    std::fill(data.begin(), data.end(), NINF);
}

Float ZBuffer::get(int i, int j) const {
    return data[width * j + i];
}

void ZBuffer::set(int i, int j, Float depth) {
    data[width * j + i] = depth;
}

ShadowBuffer::ShadowBuffer(Scene& scene, Float near, Float far)
    : scene(scene), near(near), far(far) {
}

void ShadowBuffer::addBakedShadow(int lightId, const Matrix& mvp, const TextureId& texId) {
    shadowMaps.emplace(lightId, texId);
    lightMvp.emplace(lightId, mvp);
}

bool ShadowBuffer::shadowTest(int lightId, const Vector3& pos) {
    Vector3 shadowVec = (lightMvp.at(lightId) * pos.expand(1.0f)).homoDiv();
    Float dd = -(shadowVec.z() - near) / (near - far);
    Image* shadowMap = scene.textures.get(shadowMaps.at(lightId));
    Float d = shadowMap->getPixel(shadowVec.x(), shadowVec.y()).z();
    return fabs(d - dd) < 0.01;
}

void ShadowBuffer::releaseAll() {
    for (auto [tex, _] : shadowMaps) {
        scene.textures.release(tex);
    }
    shadowMaps.clear();
    lightMvp.clear();
}

ZCPURenderer::ZCPURenderer(ZCPUConfig conf)
    : conf(conf)
    , threadPool(conf.threadNum, conf.maxWidth * conf.maxHeight)
    , shadowBuffer(scene, conf.shadowNear, conf.shadowFar) {
}

ZCPURenderer::~ZCPURenderer() {
}

Scene& ZCPURenderer::sceneRef() {
    return scene;
}

void ZCPURenderer::render(Image& screen) {
    populateShadowBuffer(screen);
    const Matrix VPOV = scene.camera.VPOV(screen);
    Shader mainPass = [&](const Vector3& bary, const Vector3& homo, const Geometry& geom,
                          Float depth) {
        if (auto triangle = geom.get<PlainTriangle>()) {
            return shadePlainTriangle(bary, *triangle);
        } else if (auto triangle = geom.get<Triangle>()) {
            return shadeTriangle(bary, *triangle, homo);
        }
    };
    renderPass(screen, VPOV, mainPass);
    shadowBuffer.releaseAll();
}

void ZCPURenderer::populateShadowBuffer(Image& screen) {
    for (auto lightId : scene.lightSystem.lights.ids()) {
        Light* l = scene.lightSystem.lights.get(lightId);
        if (auto light = l->get<DirectionalLight>()) {
            int texId = scene.textures.create(conf.shadowMapWidth, conf.shadowMapHeight);
            Basis basis;
            basis.w = light->v;
            basis.u = Vector3(0, 0, 1).cross(basis.w).normalized();
            basis.v = basis.w.cross(basis.u);
            Matrix mvp =
                viewportMatrix(conf.shadowMapWidth, conf.shadowMapHeight) *
                orthProjectionMatrix(-2.0, 2.0, -2.0, 2.0, conf.shadowNear, conf.shadowFar) *
                viewMatrix(basis, Vector3(0, 0, 0));
            shadowBuffer.addBakedShadow(lightId, mvp, texId);
            Shader shadowPass = [&](const Vector3& bary, const Vector3& homo, const Geometry& geom,
                                    Float depth) {
                return Vector3(0.0f, 0.0f,
                               -(depth - conf.shadowNear) / (conf.shadowNear - conf.shadowFar));
            };
            renderPass(*scene.textures.get(texId), mvp, shadowPass);
        }
    }
}

void ZCPURenderer::renderPass(Image& screen, const Matrix& mvp, const Shader& shader) {
    zBuffer.reset(screen.getWidth(), screen.getHeight());
    fragmentJobBuffer = threadPool.allocJobBuffer(screen.getWidth() * screen.getHeight());
    threadPool.setJobFunc([&](FragmentInfo info) {
        screen.setPixel(info.i, info.j, shader(info.bary, info.homo, info.geom, info.depth));
    });
    // Push fragment jobs into threadPool
    for (auto& [_, geom] : scene.geoms) {
        Vector3 h;
        if (auto triangle = geom.get<PlainTriangle>()) {
            if (triangle->curve.sameFace(scene.camera.basis.w)) {
                Triangle3 projected = triangle->curve.transformed(mvp, h);
                addDrawTriangleJob(screen, projected, geom, h, shader);
            }
        } else if (auto triangle = geom.get<Triangle>()) {
            if (triangle->curve.sameFace(scene.camera.basis.w)) {
                Triangle3 projected = triangle->curve.transformed(mvp, h);
                addDrawTriangleJob(screen, projected, geom, h, shader);
            }
        }
    }
    // Do all the works!
    threadPool.flush(conf.threadNum);
}

// Fast rasterization
// Barycentric cooridnates are "scaled distance" from each edge.
// a = F01(x,y)/2*area
// F01(x,y) = Ax+By+C
// F01(x+1,y) - F01(x,y) = A
// F01(x,y+1) - F01(x,y) = B
// We can do incremental update out of this
void ZCPURenderer::addDrawTriangleJob(Image& screen, const Triangle3& triangle,
                                      const Geometry& geom, const Vector3& homo,
                                      const Shader& shader) {
    // Do fixed floating point like thing to fight boundary artifact caused by "not going enough"
    constexpr Float fixedFactor = 100.0f;

    Vector2 pA = Vector2(triangle.pA[0], triangle.pA[1]);
    Vector2 pB = Vector2(triangle.pB[0], triangle.pB[1]);
    Vector2 pC = Vector2(triangle.pC[0], triangle.pC[1]);
    const Float area = (pB - pA).cross(pC - pA);
    const Float areaDiv = 1.0f / (area * fixedFactor);

    const int x0 = std::max(std::min({ pA[0], pB[0], pC[0] }), 0.0f);
    const int x1 = std::min(std::max({ pA[0], pB[0], pC[0] }) + 1, (float)screen.getWidth());
    const int y0 = std::max(std::min({ pA[1], pB[1], pC[1] }), 0.0f);
    const int y1 = std::min(std::max({ pA[1], pB[1], pC[1] }) + 1, (float)screen.getHeight());

    Line2 line01(pA, pB);
    Line2 line12(pB, pC);
    Line2 line20(pC, pA);

    const int A01 = line01.AB[0] * fixedFactor;
    const int B01 = line01.AB[1] * fixedFactor;
    const int A12 = line12.AB[0] * fixedFactor;
    const int B12 = line12.AB[1] * fixedFactor;
    const int A20 = line20.AB[0] * fixedFactor;
    const int B20 = line20.AB[1] * fixedFactor;

    Vector2 p(x0, y0);
    int w0Row = line12(p) * fixedFactor;
    int w1Row = line20(p) * fixedFactor;
    int w2Row = line01(p) * fixedFactor;

    for (int j = y0; j < y1; ++j) {
        int w0 = w0Row;
        int w1 = w1Row;
        int w2 = w2Row;
        for (int i = x0; i < x1; ++i) {
            if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                Vector3 bary(areaDiv * w0, areaDiv * w1, areaDiv * w2);
                Float depth = bary.x() * triangle.pA.z() + bary.y() * triangle.pB.z() +
                              bary.z() * triangle.pC.z();
                if (zBuffer.get(i, j) <= depth) {
                    zBuffer.set(i, j, depth);
                    fragmentJobBuffer[j * screen.getWidth() + i] = {
                        i, j, bary, homo, geom, depth
                    };
                }
            }
            w0 += A12;
            w1 += A20;
            w2 += A01;
        }
        w0Row += B12;
        w1Row += B20;
        w2Row += B01;
    }
}

Vector3 ZCPURenderer::shadePlainTriangle(const Vector3& bary, const PlainTriangle& tri) {
    Vector3 pixel = scene.lightSystem.ambientIntensity * tri.material.ambient;
    Vector3 normal = tri.normal(Vector3());
    Vector3 hit = tri.vA * bary.x() + tri.vB * bary.y() + tri.vC * bary.z();
    Vector3 e = (scene.camera.e - hit).normalized();
    for (auto& [lightId, l] : scene.lightSystem.lights) {
        Vector3 lightV;
        Float intensity;
        l.unwrap(hit, lightV, intensity);
        // if (shadowBuffer.shadowTest(lightId, hit)) {
        Vector3 h = (e + lightV).normalized();
        Float specular = std::max(0.0f, lightV.dot(normal));
        Float phong = std::max(0.0f, h.dot(normal));
        pixel += intensity * (specular * tri.material.diffuse +
                              pow(phong, tri.material.phong) * tri.material.specular);
        //}
    }
    return pixel;
}

Vector3 ZCPURenderer::shadeTriangle(const Vector3& bary, const Triangle& tri, const Vector3& homo) {
    Vector3 normal = tri.vA.normal * bary.x() + tri.vB.normal * bary.y() + tri.vC.normal * bary.z();
    Vector3 hit = tri.vA.pos * bary.x() + tri.vB.pos * bary.y() + tri.vC.pos * bary.z();
    Vector3 e = (scene.camera.e - hit).normalized();
    // Perspective correction
    Vector2 uv = tri.vA.tex * bary.x() / homo.x() + tri.vB.tex * bary.y() / homo.y() +
                 tri.vC.tex * bary.z() / homo.z();
    uv /= (bary.x() * (1 / homo.x()) + bary.y() * (1 / homo.y()) + bary.z() * (1 / homo.z()));

    Vector3 diffuse;
    if (tri.texture) {
        diffuse = sampleBilinear(*scene.textures.get(tri.texture), uv, true);
    } else {
        diffuse = tri.material.diffuse;
    }

    // Normal mapping
    if (tri.normalMap) {
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
        normal = tbn * nn;
    }

    Vector3 pixel = scene.lightSystem.ambientIntensity * diffuse;
    for (auto& [lightId, l] : scene.lightSystem.lights) {
        Vector3 lightV;
        Float intensity;
        l.unwrap(hit, lightV, intensity);
        // if (shadowBuffer.shadowTest(lightId, hit)) {
        Vector3 h = (e + lightV).normalized();
        Float specular = std::max(0.0f, lightV.dot(normal));
        Float phong = std::max(0.0f, h.dot(normal));
        pixel += intensity *
                 (specular * diffuse + pow(phong, tri.material.phong) * tri.material.specular);
        //}
    }
    return pixel;
}
