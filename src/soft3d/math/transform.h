#pragma once
#include <soft3d/math/linalg.h>

inline Float deg2rad(Float angle) {
    return angle * PI / 180.0;
}

inline Matrix rotate2(Float rad) {
    return Matrix(2, 2, { cos(rad), -sin(rad), sin(rad), cos(rad) });
}

inline Matrix shearX2(Float s) {
    return Matrix(2, 2, { 1, s, 0, 1 });
}

inline Matrix shearY2(Float s) {
    return Matrix(2, 2, { 1, 0, s, 1 });
}

inline Matrix scale2(Float sx, Float sy) {
    return Matrix(2, 2, { sx, 0, 0, sy });
}

inline Matrix scale3(Float sx, Float sy, Float sz) {
    return Matrix(3, 3, { sx, 0, 0, 0, sy, 0, 0, 0, sz });
}

inline Matrix toHomo(Matrix m) {
    Matrix out(4, 4);
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            out[mi{ i, j }] = m[mi{ i, j }];
    out[mi{ 3, 3 }] = 1.0;
    return out;
}

// [-1, 1] to [-0.5, n-0.5]
// x * (n/2) + (n-1)/2
// in matrix form
inline Matrix viewportMatrix(int width, int height) {
    return Matrix(4, 4,
                  { width / 2.0f, 0, 0, (width - 1) / 2.0f, 0, height / 2.0f, 0,
                    (height - 1) / 2.0f, 0, 0, 1, 0, 0, 0, 0, 1 });
}

// fixate z looking at the back
// Project the cube defined by (l,b,n) and (r,t,f)
// l = left plane
// r = right plane
// b = bottom plane
// t = top plane
// n = near plane
// f = far plane
// f is more negative as z is pointing backwards
// it's "windowing transformation"
// x: [l,r] to [-1,1]
// y: [b,t] to [-1,1]
// z: [n,f] to [-1,1]
// x*(2/r-l) + c
// (r+l)/2 * (2/r-1)+ c= 0
// c = -(r+l)/(r-l)
// x: x*(2/r-l)-(r+l)/(r-l)
inline Matrix orthProjectionMatrix(Float l, Float r, Float b, Float t, Float n, Float f) {
    return Matrix(4, 4,
                  { 2 / (r - l), 0, 0, -(r + l) / (r - l), 0, 2 / (t - b), 0, -(t + b) / (t - b), 0,
                    0, 2 / (n - f), -(n + f) / (n - f), 0, 0, 0, 1 });
}

// d = focal length = n
// y_s = d/z*y (sizd is proportional to 1/z)
// (x,y,z,w) => (x/w,y/w,z/w)
// the bottom row of matrix e,f,g,h -> x_w = x_o/(ex+fy+gz+h)
// it turned out it's impossible to perserve z while doing what we want
// decided to make it unchanged near n and f
// n + f - n*f/z (this actually preserve the order)
inline Matrix perspectiveProjectionMatrix(Float n, Float f) {
    return Matrix(4, 4, { n, 0, 0, 0, 0, n, 0, 0, 0, 0, n + f, -f * n, 0, 0, 1, 0 });
}

// convert world frame to camera frame
// p_xyz = basis * p_uvw = [u,v,w] * p_uvw = lin comb of columns
// p_uvw = [u,v,w]^-1 * p_xyz
// The matrix is orthogonal, [u,v,w]^-1 = [u,v,w]^T
// p_uvw = [u;v;w] * p_xyz
// T = [u;v;w]
inline Matrix viewMatrix(const Basis& basis, const Vector3& e) {
    Matrix s = Matrix(4, 4,
                      { basis.u[0], basis.u[1], basis.u[2], 0, basis.v[0], basis.v[1], basis.v[2],
                        0, basis.w[0], basis.w[1], basis.w[2], 0, 0, 0, 0, 1 });
    Matrix t = Matrix(4, 4, { 1, 0, 0, -e.x(), 0, 1, 0, -e.y(), 0, 0, 1, -e.z(), 0, 0, 0, 1 });
    return s * t;
}

// Create transformation matrix that orients vector w to vector z
// https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula
// Doesn't work this is cross poduct matrix FIXME
inline Matrix orientMatrix(const Vector3& original, const Vector3& desired) {
    Vector3 k = original.cross(desired).normalized();
    return Matrix(3, 3, { 0, -k.z(), k.y(), k.z(), 0, -k.x(), -k.y(), k.x(), 0 });
}
