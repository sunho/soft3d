#pragma once

#include <cstdlib>
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
    
    void normalize() {
        this->u() /= sqrt(this->dot(this->u()));
    }

    F& operator[](size_t index) {
        return data[index];
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
