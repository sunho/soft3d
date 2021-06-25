#pragma once

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <array>

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
    explicit GVector(const std::array<F, n>& l) : data(l) { }

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
struct GVector3 final : public GVector<GVector3<F>, F, 3> {
    GVector3() : GVector<GVector3<F>, F, 3>() {}
    explicit GVector3(F x, F y, F z) : GVector<GVector3<F>, F, 3>({x,y, z}) {}

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
};

template<typename F>
struct GVector2 final : public GVector<GVector2<F>, F, 2> {
    GVector2() : GVector<GVector2<F>, F, 2>() {}
    explicit GVector2(F x, F y) : GVector<GVector2<F>, F, 2>({x,y}) {}

    F& x() { return this->data[0]; }
    F& y() { return this->data[1]; }
    const F& x() const { return this->data[0]; }
    const F& y() const { return this->data[1]; }
};

using Float = double;
using Vector3 = GVector3<Float>;
using Vector2 = GVector2<Float>;

struct Basis {
    Vector3 u;
    Vector3 v;
    Vector3 w;
};
