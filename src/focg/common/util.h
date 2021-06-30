#pragma once

#include <focg/common/linalg.h>
#include <numeric>

constexpr Float EPSILON = std::numeric_limits<double>::epsilon();

inline bool nearGt(Float a, Float b) {
    return (a - b) > ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * EPSILON);
}

inline bool nearLt(Float a, Float b) {
    return (b - a) > ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * EPSILON);
}

inline bool nearE(Float a, Float b) {
    return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * EPSILON);
}

inline bool nearInRange(Float x, Float start, Float end) {
    return nearLt(start, x) && nearLt(x, end);
}

inline bool inRange(Float x, Float start, Float end) {
    return start <= x && x <= end;
}

inline bool inRangeExclude(Float x, Float start, Float end) {
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

template<class... Ts> struct overloaded : Ts... { using Ts::operator()...; };
template<class... Ts> overloaded(Ts...) -> overloaded<Ts...>;
