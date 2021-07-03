#pragma once

#include <nuguri3d/common/image.h>

struct Texture {
    Image image;

    Vector3 lookupClamp(const Vector2& uv) {
        int i = round(uv.x() * image.getWidth() - 0.5f);
        int j = round(uv.y() * image.getWidth() - 0.5f);
        return image.getPixel(std::max(0, std::min(i, image.getWidth() - 1)),
                              std::max(0, std::min(j, image.getHeight() - 1)));
    }
};

// convert (x,y,z) into spherical coordinates (r, theta, pi) as in physics
// (like one used in calc of angular velocity)
// discard r and map theta and pi to [0,1]
// acos: 0 to pi
// atan2: -pi to pi
// theta_map = (pi-arccos(z/sqrt(x^2+y^2+z^2)))/pi (the book flips the range first not sure why <-
// TODO) pi_map = (pi + atan2(y,x))/2pi
static Vector2 convertSphereTexcoord(const Vector3& pos) {
    Float r = pos.norm();
    Float u = (PI + atan2(pos.y(), pos.x())) / (2 * PI);
    Float v = (acos(pos.z() / r)) / PI;
    return Vector2(u, v);
}
