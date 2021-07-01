#include <focg/backend/zcpu/renderer.h>

ZCPURenderer::ZCPURenderer() {
    
}

ZCPURenderer::~ZCPURenderer() {
    
}

Scene& ZCPURenderer::sceneRef() {
    return scene;
}

void ZCPURenderer::render(Screen &screen) {
    zBuffer.resize(screen.getWidth() * screen.getHeight());
    std::fill(zBuffer.begin(), zBuffer.end(), -1.0/0.0);
    width = screen.getWidth();
    const Matrix VPOV = scene.camera.VPOV(screen);
    for (auto& geom : scene.geoms) {
       bool tmp = false;
       if(auto sphere = std::get_if<Sphere>(&geom)) {
           // todo
       } else if (auto triangle = std::get_if<Triangle>(&geom)){
           Triangle projected = triangle->transformed(VPOV);
           drawTriangle(screen, projected, *triangle);
       }
    }
}

Float ZCPURenderer::getDepth(int i, int j) {
    return zBuffer[i*width+j];
}

void ZCPURenderer::setDepth(int i, int j, Float depth) {
    zBuffer[i*width+j] = depth;
}

void ZCPURenderer::drawTriangle(Screen &screen, const Triangle &triangle, const Triangle &original) {
    Triangle2 tri(Vector2(triangle.vA[0], triangle.vA[1]), Vector2(triangle.vB[0], triangle.vB[1]), Vector2(triangle.vC[0], triangle.vC[1]));
    StdLightSystem& lightSystem = std::get<StdLightSystem>(scene.lightSystem);
    for (int i = 0; i < screen.getWidth(); ++i) {
        for (int j = 0; j < screen.getHeight(); ++j) {
            Vector3 bary = tri(Vector2(i,j));
            if (nearInRange(bary.x(), 0.0, 1.0) && nearInRange(bary.y(), 0.0, 1.0) && nearInRange(bary.z(), 0.0, 1.0)) {
                Float depth = bary.x() * triangle.vA.z() + bary.y() * triangle.vB.z() + bary.z() * triangle.vC.z();
                if (getDepth(i,j) > depth) continue;
                setDepth(i,j,depth);
                Vector3 pixel = lightSystem.ambientIntensity * triangle.shade.ambient;
                Vector3 normal = original.normal(Vector3()).normalized();
                for (auto light : lightSystem.lights) {
                    Float x = std::max(0.0, light.v.dot(normal));
                   // Float x2 = std::max(0.0, h.dot(hit.normal));
                    pixel += light.intensity * (x * triangle.shade.diffuse);
                }
                screen.setPixel(i, j, pixel);
            }
        }
    }
}
