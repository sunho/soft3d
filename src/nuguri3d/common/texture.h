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

// Just point sample it
static Vector3 samplePoint(Image& texture, const Vector2& uv) {
    int i = round(uv.x() * texture.getWidth() - 0.5f);
    int j = round(uv.y() * texture.getWidth() - 0.5f);
    return texture.getPixel(std::max(0, std::min(i, texture.getWidth() - 1)),
                            std::max(0, std::min(j, texture.getHeight() - 1)));
}

// Analogoues to image resampling
// Image coord = TexCoordFunc(uv)
// The jacobian of TextCoordFunc provides the area of TexCoord(uv)
// "distored" from xy
// proof: https://web.ma.utexas.edu/users/m408m/Display15-10-4.shtml
// We sample 4 closest texels and interpolate them using
// the jacobian components
// Q: Can interpolation be right reconsturction filter?
// A: We can apply the smooth filter on texture before going into pipeline
static Vector3 sampleBilinear(Image& texture, const Vector2& uv) {
    Float uP = uv.x() * texture.getWidth() - 0.5;
    Float vP = uv.y() * texture.getHeight() - 0.5;
    int iu0 = std::max(uP, 0.0f);
    int iu1 = iu0 + 1;
    int iv0 = std::max(vP, 0.0f);
    int iv1 = iv0 + 1;
    Float aU = (iu1 - uP);
    Float bU = (1.0f - aU);
    Float aV = (iv1 - vP);
    Float bV = (1.0f - aV);
    Vector3 out;
    out += aU * aV * texture.getPixel(iu0, iv0);
    out += aU * bV * texture.getPixel(iu0, iv1);
    out += bU * aV * texture.getPixel(iu1, iv0);
    out += bU * bV * texture.getPixel(iu1, iv1);
    return out;
}
