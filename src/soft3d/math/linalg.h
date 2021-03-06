#pragma once

#include <functional>
#include <cassert>
#include <array>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <sstream>
#include <vector>
#include <algorithm>

namespace details {
template <typename T, typename F>
struct ctrp {
  protected:
    T& u() {
        return static_cast<T&>(*this);
    }

    const T& u() const {
        return static_cast<const T&>(*this);
    }
};
}  // namespace details

template <typename T, typename F, size_t n>
struct GVector : public details::ctrp<T, F> {
    GVector() {
        std::fill_n(data, 4, 0.0f);
    }
    explicit GVector(const F* l) {
        for (size_t i = 0; i < 4; ++i) {
            if (i < n) {
                data[i] = l[i];
            } else {
                data[i] = 0.0f;
            }
        }
    }
    explicit GVector(const std::vector<F>& l) {
        for (size_t i = 0; i < 4; ++i) {
            if (i < n) {
            data[i] = l[i];
            } else {
                data[i] = 0.0f;
            }
        }
    }
    
    friend T operator-(const T& obj) {
        return -1 * obj;
    }

    friend inline T operator/(F scalar, const T& vec) {
        T out = vec;
        for (size_t i = 0; i < n; ++i) {
            out[i] = scalar / out[i];
        }
        return out;
    }

    friend inline T operator*(F scalar, const T& vec) {
        T out = vec;
        out *= scalar;
        return out;
    }

    inline T& operator+=(const T& other) {
        for (size_t i = 0; i < n; ++i) {
            data[i] += other.data[i];
        }
        return this->u();
    }

    inline T operator+(const T& other) const {
        T out = this->u();
        out += other;
        return out;
    }

    inline T& operator-=(const T& other) {
        for (size_t i = 0; i < n; ++i) {
            data[i] -= other.data[i];
        }
        return this->u();
    }

    inline T operator-(const T& other) const {
        T out = this->u();
        out -= other;
        return out;
    }

    inline T& operator*=(const T& other) {
        for (size_t i = 0; i < n; ++i) {
            data[i] *= other.data[i];
        }
        return this->u();
    }

    inline T operator*(const T& other) const {
        T out = this->u();
        out *= other;
        return out;
    }

    inline T& operator%=(const T& other) {
        for (size_t i = 0; i < n; ++i) {
            data[i] = fmod(data[i], other.data[i]);
        }
        return this->u();
    }

    inline T operator%(const T& other) const {
        T out = this->u();
        out %= other;
        return out;
    }

    inline T& operator%=(F scalar) {
        for (size_t i = 0; i < n; ++i) {
            data[i] = fmod(data[i], scalar);
        }
        return this->u();
    }

    inline T operator%(F scalar) const {
        T out = *this;
        out %= scalar;
        return out;
    }

    inline T& operator*=(F scalar) {
        for (size_t i = 0; i < n; ++i) {
            data[i] *= scalar;
        }
        return this->u();
    }

    inline T operator*(F scalar) const {
        T out = this->u();
        out *= scalar;
        return out;
    }

    inline T& operator-=(F scalar) {
        for (size_t i = 0; i < n; ++i) {
            data[i] -= scalar;
        }
        return this->u();
    }

    inline T operator-(F scalar) const {
        T out = this->u();
        out -= scalar;
        return out;
    }

    inline T& operator+=(F scalar) {
        for (size_t i = 0; i < n; ++i) {
            data[i] += scalar;
        }
        return this->u();
    }

    inline T operator+(F scalar) const {
        T out = this->u();
        out += scalar;
        return out;
    }

    inline T& operator/=(F scalar) {
        for (size_t i = 0; i < n; ++i) {
            data[i] /= scalar;
        }
        return this->u();
    }

    inline T operator/(F scalar) const {
        T out = this->u();
        out /= scalar;
        return out;
    }

    inline T& operator/=(const T& other) {
        for (size_t i = 0; i < n; ++i) {
            data[i] /= other[i];
        }
        return this->u();
    }

    inline T operator/(const T& other) const {
        T out = this->u();
        out /= other;
        return out;
    }
    
    bool hasNan() const {
        for (size_t i = 0; i < n; ++i) {
            if (isnan(data[i]) || !isfinite(data[i])) {
                return true;
            }
        }
        return false;
    }

    inline F dot(const T& other) const {
        T tmp = this->u();
        tmp *= other;
        F out = 0;
        for (size_t i = 0; i < n; ++i) {
            out += tmp[i];
        }
        return out;
    }

    inline T max(const T& other) const {
        T out = this->u();
        for (size_t i = 0; i < n; ++i) {
            out[i] = std::max(out[i], other[i]);
        }
        return out;
    }

    inline T min(const T& other) const {
        T out = this->u();
        for (size_t i = 0; i < n; ++i) {
            out[i] = std::min(out[i], other[i]);
        }
        return out;
    }

    F hmax() const {
        return *std::max_element(data, data + n);
    }

    F hmin() const {
        return *std::min_element(data, data + n);
    }

    inline F norm2() const {
        return this->dot(this->u());
    }

    inline F norm() const {
        return sqrt(norm2());
    }

    inline void normalize() {
        this->u() /= sqrt(this->dot(this->u()));
    }

    inline T normalized() const {
        T out = this->u();
        out.normalize();
        return out;
    }

    F& operator[](size_t index) {
        return data[index];
    }

    const F& operator[](size_t index) const {
        return data[index];
    }

    std::string desc() const {
        std::stringstream ss("");
        ss << "(";
        for (size_t i = 0; i < n; ++i) {
            ss << data[i];
            if (i != n - 1) {
                ss << ", ";
            }
        }
        ss << ")";
        return ss.str();
    }

  protected:
    F data[4];
};

template <typename F>
struct GVector4;

template <typename F>
struct GMatrix;

constexpr float EPSILON = 1e-8;
constexpr float ZERO_EPSILON = 1e-4;

template <typename F>
struct GVector3 final : public GVector<GVector3<F>, F, 3> {
    GVector3() : GVector<GVector3<F>, F, 3>() {
    }
    explicit GVector3(F x, F y, F z) : GVector<GVector3<F>, F, 3>({ x, y, z }) {
    }
    explicit GVector3(const F* l) : GVector<GVector3<F>, F, 3>(l) {
    }
    explicit GVector3(const std::vector<F>& l) : GVector<GVector3<F>, F, 3>(l) {
    }
    explicit GVector3(const uint32_t hex) : GVector<GVector3<F>, F, 3>() {
        uint8_t r = (hex >> 16) & 0xFF;
        uint8_t g = (hex >> 8) & 0xFF;
        uint8_t b = hex & 0xFF;
        this->data[0] = r / 255.0;
        this->data[1] = g / 255.0;
        this->data[2] = b / 255.0;
    }

    F& x() {
        return this->data[0];
    }
    F& y() {
        return this->data[1];
    }
    F& z() {
        return this->data[2];
    }

    inline GVector3<F> cross(const GVector3<F>& other) const {
        GVector3<F> out;
        out.x() = y() * other.z() - z() * other.y();
        out.y() = z() * other.x() - x() * other.z();
        out.z() = x() * other.y() - y() * other.x();
        return out;
    }

    const F& x() const {
        return this->data[0];
    }

    bool isZero() const {
        return fabs(x()) <= ZERO_EPSILON && fabs(y()) <= ZERO_EPSILON && fabs(z()) <= ZERO_EPSILON;
    }
    const F& y() const {
        return this->data[1];
    }
    const F& z() const {
        return this->data[2];
    }

    GVector4<F> expand(F w) const;
};

template <typename F>
struct GVector4 final : public GVector<GVector4<F>, F, 4> {
    GVector4() : GVector<GVector4<F>, F, 4>() {
    }
    explicit GVector4(F x, F y, F z, F w) : GVector<GVector4<F>, F, 4>({ x, y, z, w }) {
    }
    explicit GVector4(const F* l) : GVector<GVector4<F>, F, 4>(l) {
    }
    explicit GVector4(const std::vector<F>& l) : GVector<GVector4<F>, F, 4>(l) {
    }

    F& x() {
        return this->data[0];
    }
    F& y() {
        return this->data[1];
    }
    F& z() {
        return this->data[2];
    }
    F& w() {
        return this->data[3];
    }

    const F& x() const {
        return this->data[0];
    }
    const F& y() const {
        return this->data[1];
    }
    const F& z() const {
        return this->data[2];
    }
    const F& w() const {
        return this->data[3];
    }

    GVector3<F> trunc() const {
        return GVector3<F>( this->data[0], this->data[1], this->data[2] );
    }

    GVector3<F> homoDiv() const {
        if (w() == 0.0f) {
            return GVector3<F>( this->data[0], this->data[1], this->data[2] );
        }
        return GVector3<F>( this->data[0] / w(), this->data[1] / w(), this->data[2] / w() );
    }
};

template <typename F>
GVector4<F> GVector3<F>::expand(F w) const {
    return GVector4<F>( this->data[0], this->data[1], this->data[2], w);
}

template <typename F>
struct GVector2 final : public GVector<GVector2<F>, F, 2> {
    GVector2() : GVector<GVector2<F>, F, 2>() {
    }
    explicit GVector2(F x, F y) : GVector<GVector2<F>, F, 2>({ x, y }) {
    }
    explicit GVector2(const F* l) : GVector<GVector2<F>, F, 2>(l) {
    }
    explicit GVector2(const std::vector<F>& l) : GVector<GVector2<F>, F, 2>(l) {
    }

    F& x() {
        return this->data[0];
    }
    F& y() {
        return this->data[1];
    }

    const F& x() const {
        return this->data[0];
    }
    const F& y() const {
        return this->data[1];
    }

    // set z = 0 and do the thing
    F cross(const GVector2<F>& other) const {
        return x() * other.y() - other.x() * y();
    }
};

struct GMatrixIndex {
    size_t i;
    size_t j;
};

using mi = GMatrixIndex;

using Float = float;
using Vector4 = GVector4<Float>;
using Vector3 = GVector3<Float>;
using Vector2 = GVector2<Float>;

template <typename F>
struct GMatrix {
    GMatrix() = default;
    explicit GMatrix(size_t height, size_t width)
        : width(width), height(height), data(width * height) {
        std::fill(data.begin(), data.end(), 0);
    }
    explicit GMatrix(size_t height, size_t width, const std::vector<F>& l)
        : width(width), height(height), data(l) {
    }

    explicit GMatrix(size_t height, size_t width, const F* l)
        : width(width), height(height), data(width * height) {
        for (int i = 0; i < width * height; ++i)
            data[i] = l[i];
    }

    F& operator[](GMatrixIndex index) {
        return data[index.i * width + index.j];
    }

    const F& operator[](GMatrixIndex index) const {
        return data[index.i * width + index.j];
    }

    F& operator[](size_t index) {
        return data[index];
    }

    const F& operator[](size_t index) const {
        return data[index];
    }

    GMatrix<F> operator*(const GMatrix<F>& other) const {
        assert(width == other.height);
        GMatrix<F> out(height, other.width);
        for (size_t i = 0; i < out.height; ++i) {
            for (size_t j = 0; j < out.width; ++j) {
                for (size_t t = 0; t < width; ++t) {
                    out[mi{ i, j }] += (*this)[mi{ i, t }] * other[mi{ t, j }];
                }
            }
        }
        return out;
    }

    GMatrix<F> transpose() const {
        GMatrix<F> out(width, height);
        for (size_t i = 0; i < height; ++i) {
            for (size_t j = 0; j < width; ++j) {
                out[mi{ j, i }] = (*this)[mi{ i, j }];
            }
        }
        return out;
    }

#define VECTOR_OP_IMPL(Vec, Num)                                                                   \
    Vec operator*(const Vec& other) const {                                                        \
        return mul<Vec>(other);                                                                    \
    }                                                                                              \
    static GMatrix<F> stackRowVector##Num(std::initializer_list<Vec> vecs) {                       \
        return stackRowVectors<Vec, Num>(vecs);                                                    \
    }                                                                                              \
    static GMatrix<F> stackColVector##Num(std::initializer_list<Vec> vecs) {                       \
        return stackColVectors<Vec, Num>(vecs);                                                    \
    }                                                                                              \
    Vec sliceColVector##Num(size_t index) const {                                                  \
        return sliceColVector<Vec>(index);                                                         \
    }                                                                                              \
    Vec sliceRowVector##Num(size_t index) const {                                                  \
        return sliceRowVector<Vec>(index);                                                         \
    }

    VECTOR_OP_IMPL(Vector2, 2)
    VECTOR_OP_IMPL(Vector3, 3)
    VECTOR_OP_IMPL(Vector4, 4)
#undef VECTOR_OP_IMPL

  private:
    template <typename OVec, typename IVec>
    OVec mul(const IVec& other) const {
        OVec out;
        for (size_t i = 0; i < height; ++i) {
            for (size_t t = 0; t < width; ++t) {
                out[i] += (*this)[mi{ i, t }] * other[t];
            }
        }
        return out;
    }

    template <typename Vec, size_t Num>
    static GMatrix<F> stackRowVectors(std::initializer_list<Vec> vecs) {
        GMatrix<F> out(vecs.size(), Num);
        size_t j = 0;
        for (auto& vec : vecs) {
            for (size_t i = 0; i < Num; ++i) {
                out[mi{ i, j }] = vec[i];
            }
            ++j;
        }
        return out;
    }

    template <typename Vec, size_t Num>
    static GMatrix<F> stackColVectors(const std::initializer_list<Vec> vecs) {
        GMatrix<F> out(Num, vecs.size());
        size_t i = 0;
        for (auto& vec : vecs) {
            for (size_t j = 0; j < Num; ++i) {
                out[mi{ i, j }] = vec[j];
            }
            ++i;
        }
        return out;
    }

    template <typename Vec>
    Vec sliceColVector(size_t index) const {
        std::vector<F> l;
        for (int i = 0; i < height; ++i) {
            l.push_back(data[i * width + index]);
        }
        return Vec(l);
    }

    template <typename Vec>
    Vec sliceRowVector(size_t index) const {
        return Vec(data.data() + index * height);
    }

    size_t width{ 0 };
    size_t height{ 0 };
    std::vector<F> data;
};

using Matrix = GMatrix<Float>;

constexpr Float PI = 3.14159265358979323846f;
static const Float INF = std::numeric_limits<float>::infinity();
static const Float NINF = -std::numeric_limits<float>::infinity();

struct Basis {
    Basis() = default;
    Basis(const Matrix& mat)
        : u(mat.sliceColVector3(0)), v(mat.sliceColVector3(1)), w(mat.sliceColVector3(2)) {
    }
    Basis(Vector3 w) : w(w) {
        if (abs(w.x()) > abs(w.y())) {
            u = Vector3(-w.z(), 0.0f, w.x()) / sqrt(w.x() * w.x() + w.z() * w.z());
        } else {
            u = Vector3(0.0f, w.z(), -w.y()) / sqrt(w.y() * w.y() + w.z() * w.z());
        }
        v = w.cross(u);
    }  
    Basis(Vector3 u, Vector3 v, Vector3 w) : u(u), v(v), w(w) {
    }

    Vector3 fromGlobal(Vector3 k) {
        return Vector3(u.dot(k), v.dot(k), w.dot(k));
    }

    Vector3 toGlobal(Vector3 k) {
        return k.x() * u + k.y() * v + k.z() * w;
    }

    Vector3 u{ 1.0f, 0.0f, 0.0f };
    Vector3 v{ 0.0f, 1.0f, 0.0f };
    Vector3 w{ 0.0f, 0.0f, 1.0f };

    Matrix asMatrix() const {
        return Matrix(3, 3, { u[0], v[0], w[0], u[1], v[1], w[1], u[2], v[2], w[2] });
    }
};

static Vector3 sphericalDir(Float cosTh, Float sinTh, Float phi) {
    return Vector3(sinTh * cos(phi), sinTh * sin(phi), cosTh);
}


static Matrix I4x4 = Matrix(4, 4, { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 });
