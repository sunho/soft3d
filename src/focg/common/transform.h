#pragma once
#include <focg/common/linalg.h>

inline Float deg2rad(Float angle) {
    return angle * PI / 180.0;
}

inline Matrix rotate2(Float rad) {
    return Matrix(2, 2, {cos(rad), -sin(rad), sin(rad), cos(rad)});
}

inline Matrix shearX2(Float s) {
    return Matrix(2, 2, {1, s, 0, 1});
}

inline Matrix shearY2(Float s) {
    return Matrix(2, 2, {1, 0, s, 1});
}

inline Matrix scale2(Float sx, Float sy) {
    return Matrix(2, 2, {sx, 0, 0, sy});
}

inline Matrix scale3(Float sx, Float sy, Float sz) {
    return Matrix(3, 3, {sx, 0, 0, 0, sy, 0, 0, 0, sz});
}

inline Matrix toHomo(Matrix m) {
    Matrix out(4,4);
    for (size_t i = 0; i < 3; ++i)
        for (size_t j = 0; j < 3; ++j)
            out[mi{i,j}] = m[mi{i,j}];
    out[mi{3,3}] = 1.0;
    return out;
}
