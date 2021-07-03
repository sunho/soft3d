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
        if (auto sphere = std::get_if<Sphere>(&geom)) {
            // todo
        } else if (auto triangle = std::get_if<Triangle>(&geom)) {
            Triangle projected = triangle->transformed(VPOV);
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

void ZCPURenderer::drawTriangle(Image &screen, const Triangle &triangle, const Triangle &original) {
    Triangle2 tri(Vector2(triangle.vA[0], triangle.vA[1]), Vector2(triangle.vB[0], triangle.vB[1]),
                  Vector2(triangle.vC[0], triangle.vC[1]));
    StdLightSystem &lightSystem = std::get<StdLightSystem>(scene.lightSystem);
    const int x0 = std::max(std::min({ triangle.vA[0], triangle.vB[0], triangle.vC[0] }), 0.0f);
    const int x1 = std::min(std::max({ triangle.vA[0], triangle.vB[0], triangle.vC[0] }) + 1,
                            (float)screen.getWidth());
    const int y0 = std::max(std::min({ triangle.vA[1], triangle.vB[1], triangle.vC[1] }), 0.0f);
    const int y1 = std::min(std::max({ triangle.vA[1], triangle.vB[1], triangle.vC[1] }) + 1,
                            (float)screen.getHeight());
    for (int j = y0; j < y1; ++j) {
        for (int i = x0; i < x1; ++i) {
            Vector3 bary = tri(Vector2(i, j));
            if (nearInRange(bary.x(), 0.0, 1.0) && nearInRange(bary.y(), 0.0, 1.0) &&
                nearInRange(bary.z(), 0.0, 1.0)) {
                Float depth = bary.x() * triangle.vA.z() + bary.y() * triangle.vB.z() +
                              bary.z() * triangle.vC.z();
                if (getDepth(i, j) > depth)
                    continue;
                setDepth(i, j, depth);
                Vector3 pixel = lightSystem.ambientIntensity * triangle.shade.ambient;
                Vector3 normal = original.normal(Vector3()).normalized();
                for (auto light : lightSystem.lights) {
                    Float x = std::max(0.0f, light.v.dot(normal));
                    // Float x2 = std::max(0.0, h.dot(hit.normal));
                    pixel += light.intensity * (x * triangle.shade.diffuse);
                }
                screen.setPixel(i, j, pixel);
            }
        }
    }
}
