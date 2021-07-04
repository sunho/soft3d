#pragma once

#include <nuguri3d/common/image.h>

using TextureId = size_t;

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
    Float v = (asin(pos.z() / r)) / PI;
    return Vector2(u, v);
}
