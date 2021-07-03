#include <nuguri3d/backend/zcpu/renderer.h>

ZCPURenderer::ZCPURenderer() {
}

ZCPURenderer::~ZCPURenderer() {
}

Scene &ZCPURenderer::sceneRef() {
    return scene;
}

void ZCPURenderer::render(Image &screen) {
    zBuffer.resize(screen.getWidth() * screen.getHeight());
    std::fill(zBuffer.begin(), zBuffer.end(), -1.0 / 0.0);
    width = screen.getWidth();
    const Matrix VPOV = scene.camera.VPOV(screen);
    for (auto &geom : scene.geoms) {
        bool tmp = false;
        if (auto sphere = std::get_if<PlainSphere>(&geom)) {
            // todo
        } else if (auto triangle = std::get_if<PlainTriangle>(&geom)) {
            Triangle3 projected = triangle->curve.transformed(VPOV);
            drawTriangle(screen, projected, *triangle);
        } else if (auto triangle = std::get_if<Triangle>(&geom)) {
            Triangle3 projected = triangle->curve.transformed(VPOV);
            drawTriangle(screen, projected, *triangle);
        }
    }
}

Float ZCPURenderer::getDepth(int i, int j) {
    return zBuffer[j * width + i];
}

void ZCPURenderer::setDepth(int i, int j, Float depth) {
    zBuffer[j * width + i] = depth;
}

void ZCPURenderer::drawTriangle(Image &screen, const Triangle3 &triangle, const Geometry &geom) {
    Triangle2 tri(Vector2(triangle.pA[0], triangle.pA[1]), Vector2(triangle.pB[0], triangle.pB[1]),
                  Vector2(triangle.pC[0], triangle.pC[1]));
    const int x0 = std::max(std::min({ triangle.pA[0], triangle.pB[0], triangle.pC[0] }), 0.0f);
    const int x1 = std::min(std::max({ triangle.pA[0], triangle.pB[0], triangle.pC[0] }) + 1,
                            (float)screen.getWidth());
    const int y0 = std::max(std::min({ triangle.pA[1], triangle.pB[1], triangle.pC[1] }), 0.0f);
    const int y1 = std::min(std::max({ triangle.pA[1], triangle.pB[1], triangle.pC[1] }) + 1,
                            (float)screen.getHeight());

    std::function<Vector3(const Vector3 &)> shader;
    if (auto triangle = std::get_if<PlainTriangle>(&geom)) {
        shader = [=](const Vector3 &bary) {
            return this->shadeSingleShadedTriangle(bary, *triangle);
        };
    } else if (auto triangle = std::get_if<Triangle>(&geom)) {
        shader = [=](const Vector3 &bary) { return this->shadeTriangle(bary, *triangle); };
    }
    for (int j = y0; j < y1; ++j) {
        for (int i = x0; i < x1; ++i) {
            Vector3 bary = tri(Vector2(i, j));
            if (nearInRange(bary.x(), 0.0, 1.0) && nearInRange(bary.y(), 0.0, 1.0) &&
                nearInRange(bary.z(), 0.0, 1.0)) {
                Float depth = bary.x() * triangle.pA.z() + bary.y() * triangle.pB.z() +
                              bary.z() * triangle.pC.z();
                if (getDepth(i, j) > depth)
                    continue;
                setDepth(i, j, depth);
                screen.setPixel(i, j, shader(bary));
            }
        }
    }
}

Vector3 ZCPURenderer::shadeSingleShadedTriangle(const Vector3 &bary, const PlainTriangle &tri) {
    StdLightSystem &lightSystem = std::get<StdLightSystem>(scene.lightSystem);
    Vector3 pixel = lightSystem.ambientIntensity * tri.shade.ambient;
    Vector3 normal = tri.normal(Vector3());
    for (auto light : lightSystem.lights) {
        Float x = std::max(0.0f, light.v.dot(normal));
        // Float x2 = std::max(0.0, h.dot(hit.normal));
        pixel += light.intensity * (x * tri.shade.diffuse);
    }
    return pixel;
}

Vector3 ZCPURenderer::shadeTriangle(const Vector3 &bary, const Triangle &tri) {
    StdLightSystem &lightSystem = std::get<StdLightSystem>(scene.lightSystem);
    Vector3 pixel = lightSystem.ambientIntensity * tri.shade.ambient;
    Vector3 hit = tri.vA.pos * bary.x() + tri.vB.pos * bary.y() + tri.vC.pos * bary.z();
    Vector3 e = (scene.camera.e - hit).normalized();
    Vector3 normal = tri.vA.normal * bary.x() + tri.vB.normal * bary.y() + tri.vC.normal * bary.z();
    for (auto light : lightSystem.lights) {
        Vector3 h = (e + light.v).normalized();

        Float x = std::max(0.0f, light.v.dot(normal));
        Float x2 = std::max(0.0f, h.dot(normal));
        pixel += light.intensity *
                 (x * tri.shade.diffuse + pow(x2, tri.shade.phong) * tri.shade.specular);
    }
    return pixel;
}
