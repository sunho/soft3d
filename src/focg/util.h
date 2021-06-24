#pragma once
#include <focg/geom.h>
#include <numeric>

constexpr Float EPSILON = std::numeric_limits<double>::epsilon();

bool nearGt(Float a, Float b) {
    return (a - b) > ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * EPSILON);
}

bool nearLt(Float a, Float b) {
    return (b - a) > ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * EPSILON);
}

bool nearE(Float a, Float b) {
    return fabs(a - b) <= ( (fabs(a) < fabs(b) ? fabs(b) : fabs(a)) * EPSILON);
}

bool nearInRange(Float x, Float start, Float end) {
    return nearLt(start, x) && nearLt(x, end);
}

bool inRange(Float x, Float start, Float end) {
    return start <= x && x <= end;
}
