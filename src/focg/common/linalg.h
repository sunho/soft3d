#pragma once

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <array>
#include <optional>
#include <vector>

namespace details {
    template<typename T, typename F>
    struct ctrp {
    protected:
        T& u() {
            return static_cast<T&>(*this);
        }

        const T& u() const {
            return static_cast<const T&>(*this);
        }
    };
}

template<typename T, typename F, size_t n>
struct GVector : public details::ctrp<T, F> {
    GVector() {
        data.fill(0);
    }
    explicit GVector(const F* l) : data(l) { }
    explicit GVector(const std::vector<F>& l) {
        for (size_t i = 0; i < n; ++i) {
            data[i] = l[i];
        }
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
    
    inline F dot(const T& other) const {
        T tmp = this->u();
        tmp *= other;
        F out = 0;
        for (size_t i = 0; i < n; ++i) { 
            out += tmp[i];
        }
        return out;
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
            if (i != n-1) {
                ss << ", ";
            }
        }
        ss << ")";
        return ss.str();
    }

protected:
    std::array<F, n> data;
};

template<typename F>
struct GVector4;

template<typename F>
struct GMatrix;

template<typename F>
struct GVector3 final : public GVector<GVector3<F>, F, 3> {
    GVector3() : GVector<GVector3<F>, F, 3>() {}
    explicit GVector3(F x, F y, F z) : GVector<GVector3<F>, F, 3>({x,y, z}) {}
    explicit GVector3(const F* l) : GVector<GVector3<F>, F, 3>(l) {}
    explicit GVector3(const std::vector<F>& l) : GVector<GVector3<F>, F, 3>(l) {}

    F& x() { return this->data[0]; }
    F& y() { return this->data[1]; }
    F& z() { return this->data[2]; }
    
    inline GVector3<F> cross(const GVector3<F>& other) const {
        GVector3<F> out;
        out.x() = y() * other.z() - z() * other.y();
        out.y() = z() * other.x() - x() * other.z();
        out.z() = x() * other.y() - y() * other.x();
        return out;
    }
    
    const F& x() const { return this->data[0]; }
    const F& y() const { return this->data[1]; }
    const F& z() const { return this->data[2]; }
    
    GVector4<F> expand(F w) const;
    GVector3<F> transformed(const GMatrix<F>& transform, F w) const;
};

template<typename F>
struct GVector4 final : public GVector<GVector4<F>, F, 4> {
    GVector4() : GVector<GVector4<F>, F, 4>() {}
    explicit GVector4(F x, F y, F z, F w) : GVector<GVector4<F>, F, 4>({x,y,z,w}) {}
    explicit GVector4(const F* l) : GVector<GVector4<F>, F, 4>(l) {}
    explicit GVector4(const std::vector<F>& l) : GVector<GVector4<F>, F, 4>(l) {}

    F& x() { return this->data[0]; }
    F& y() { return this->data[1]; }
    F& z() { return this->data[2]; }
    F& w() { return this->data[3]; }

    const F& x() const { return this->data[0]; }
    const F& y() const { return this->data[1]; }
    const F& z() const { return this->data[2]; }
    const F& w() const { return this->data[3]; }
    
    GVector3<F> trunc() const {
        return GVector3<F>({this->data[0], this->data[1], this->data[2]});
    }
};

template<typename F>
GVector4<F> GVector3<F>::expand(F w) const {
    return GVector4<F>({this->data[0], this->data[1], this->data[2], w});
}

template<typename F>
struct GVector2 final : public GVector<GVector2<F>, F, 2> {
    GVector2() : GVector<GVector2<F>, F, 2>() {}
    explicit GVector2(F x, F y) : GVector<GVector2<F>, F, 2>({x,y}) {}
    explicit GVector2(const F* l) : GVector<GVector2<F>, F, 2>(l) {}
    explicit GVector2(const std::vector<F>& l) : GVector<GVector2<F>, F, 2>(l) {}

    F& x() { return this->data[0]; }
    F& y() { return this->data[1]; }
    const F& x() const { return this->data[0]; }
    const F& y() const { return this->data[1]; }
};

struct GMatrixIndex {
    size_t i;
    size_t j;
};

using mi = GMatrixIndex;

template<typename F>
struct GMatrix {
    GMatrix() = default;
    explicit GMatrix(size_t height, size_t width) : width(width), height(height), data(width*height) {
        std::fill(data.begin(), data.end(), 0);
    }
    explicit GMatrix(size_t height, size_t width, const std::vector<F>& l) : width(width), height(height), data(l) { }

    explicit GMatrix(size_t height, size_t width, const F* l) : width(width), height(height), data(width*height) {
        for (int i = 0; i < width*height; ++i) data[i] = l[i];
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

    template<typename Vec>
    Vec sliceColumnV(size_t index) {
        std::vector<F> l;
        for (int i = 0; i < height; ++i) {
            l.push_back(data[i*width + index]);
        }
        return Vec(l);
    }
    
    template<typename Vec>
    Vec sliceRowV(size_t index) {
        return Vec(data.data() + index * height);
    }
    
    GMatrix<F> operator*(const GMatrix<F>& other) {
        assert(width == other.height);
        GMatrix<F> out(height, other.width);
        for (size_t i = 0; i < out.height; ++i) {
            for (size_t j = 0; j < out.width; ++j) {
                for (size_t t = 0; t < width; ++t) {
                    out[mi{i,j}] += (*this)[mi{i,t}] * other[mi{t,j}];
                }
            }
        }
        return out;
    }
    
    template<typename OVec, typename IVec>
    OVec mul(const IVec& other) const {
        OVec out;
        for (size_t i = 0; i < height; ++i) {
            for (size_t t = 0; t < width; ++t) {
               out[i] += (*this)[mi{i,t}] * other[t];
            }
        }
        return out;
    }
    
    GMatrix<F> transpose() const {
        GMatrix<F> out(width, height);
        for (size_t i = 0; i < height; ++i) {
            for (size_t j = 0; j < width; ++j) {
                out[mi{j,i}] = (*this)[mi{i,j}];
            }
        }
        return out;
    }
private:
    size_t width{0};
    size_t height{0};
    std::vector<F> data;
};

template<typename F>
GVector3<F> GVector3<F>::transformed(const GMatrix<F>& transform, F w) const {
    GVector4<F> res = transform.template mul<GVector4<F>>(expand(w));
    return res.trunc();
}

using Float = double;
using Vector4 = GVector4<Float>;
using Vector3 = GVector3<Float>;
using Vector2 = GVector2<Float>;
using Matrix = GMatrix<Float>;

constexpr Float PI = 3.14159265358979323846;


struct Basis {
    Vector3 u;
    Vector3 v;
    Vector3 w;
};

static Matrix I4x4 = Matrix(4,4,{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1});
static std::optional<Matrix> invertMatrix4x4(const Matrix& m) {
    Float inv[16], det;
    int i;
    
    inv[0] = m[5]  * m[10] * m[15] -
             m[5]  * m[11] * m[14] -
             m[9]  * m[6]  * m[15] +
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] -
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] +
              m[4]  * m[11] * m[14] +
              m[8]  * m[6]  * m[15] -
              m[8]  * m[7]  * m[14] -
              m[12] * m[6]  * m[11] +
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] -
             m[4]  * m[11] * m[13] -
             m[8]  * m[5] * m[15] +
             m[8]  * m[7] * m[13] +
             m[12] * m[5] * m[11] -
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] +
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] -
               m[8]  * m[6] * m[13] -
               m[12] * m[5] * m[10] +
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] +
              m[1]  * m[11] * m[14] +
              m[9]  * m[2] * m[15] -
              m[9]  * m[3] * m[14] -
              m[13] * m[2] * m[11] +
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] -
             m[0]  * m[11] * m[14] -
             m[8]  * m[2] * m[15] +
             m[8]  * m[3] * m[14] +
             m[12] * m[2] * m[11] -
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] +
              m[0]  * m[11] * m[13] +
              m[8]  * m[1] * m[15] -
              m[8]  * m[3] * m[13] -
              m[12] * m[1] * m[11] +
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] -
              m[0]  * m[10] * m[13] -
              m[8]  * m[1] * m[14] +
              m[8]  * m[2] * m[13] +
              m[12] * m[1] * m[10] -
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] -
             m[1]  * m[7] * m[14] -
             m[5]  * m[2] * m[15] +
             m[5]  * m[3] * m[14] +
             m[13] * m[2] * m[7] -
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] +
              m[0]  * m[7] * m[14] +
              m[4]  * m[2] * m[15] -
              m[4]  * m[3] * m[14] -
              m[12] * m[2] * m[7] +
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] -
              m[0]  * m[7] * m[13] -
              m[4]  * m[1] * m[15] +
              m[4]  * m[3] * m[13] +
              m[12] * m[1] * m[7] -
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] +
               m[0]  * m[6] * m[13] +
               m[4]  * m[1] * m[14] -
               m[4]  * m[2] * m[13] -
               m[12] * m[1] * m[6] +
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] +
              m[1] * m[7] * m[10] +
              m[5] * m[2] * m[11] -
              m[5] * m[3] * m[10] -
              m[9] * m[2] * m[7] +
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] -
             m[0] * m[7] * m[10] -
             m[4] * m[2] * m[11] +
             m[4] * m[3] * m[10] +
             m[8] * m[2] * m[7] -
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] +
               m[0] * m[7] * m[9] +
               m[4] * m[1] * m[11] -
               m[4] * m[3] * m[9] -
               m[8] * m[1] * m[7] +
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] -
              m[0] * m[6] * m[9] -
              m[4] * m[1] * m[10] +
              m[4] * m[2] * m[9] +
              m[8] * m[1] * m[6] -
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return std::nullopt;

    det = 1.0 / det;

    Matrix out(4,4);
    for (i = 0; i < 16; i++)
        out[i] = inv[i] * det;

    return out;
}
