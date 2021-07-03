#pragma once

#include <nuguri3d/common/linalg.h>

#include <ctime>
#include <numeric>

constexpr Float EPSILON = 1e-8;

inline bool nearE(Float a, Float b) {
    return fabs(a - b) < EPSILON;
}

inline bool nearGte(Float a, Float b) {
    return a > b || nearE(a, b);
}

inline bool nearLte(Float a, Float b) {
    return a < b || nearE(a, b);
}

inline bool nearInRange(Float x, Float start, Float end) {
    return nearLte(start, x) && nearLte(x, end);
}

inline bool inRange(Float x, Float start, Float end) {
    return start < x && x < end;
}

inline Float clamp(Float x, Float min, Float max) {
    if (x < min) {
        return min;
    }
    if (x > max) {
        return max;
    }
    return x;
}

inline Float clampToNormal(Float x) {
    return clamp(x, 0.0, 1.0);
}

inline double clockToMs(clock_t ticks) {
    return (ticks / (double)CLOCKS_PER_SEC) * 1000.0;
}

template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;
