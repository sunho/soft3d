#pragma once
#include <cmath>
#include <focg/util.h>
#include <focg/geom.h>

// f(x,y) = Ax + By + C
// f(x,y) = (y0 - y1)x + (x1 - x0)y + x0y1 - x1y0 = 0
// distance = k sqrt(A^2 + B^2) = f(x,y) / sqrt(A^2 + B^2)
struct Line2 {
    Vector2 AB{};
    Float C {0.0};
    Float normalizer {0.0};

    Line2() = default;
    explicit Line2(const Vector2& a, const Vector2& b) {
        AB = Vector2(a.y() - b.y(), b.x() - a.x());
        C = a.x()*b.y() - b.x()*a.y();
        normalizer = sqrt(a.dot(a) + b.dot(b));
    }

    Float operator()(const Vector2& p) {
        return AB.dot(p) + C;
    }
    
    Float distance(const Vector2& p) {
         return (*this)(p) / normalizer;
    }
};

// barycentric beta is the signed scaled distance from the l_AC
// beta = f_ac (x, y) / f_ac(x_b, y_b)
struct Triangle {
    Line2 acL;
    Line2 abL;
    Float fB {0.0};
    Float fC {0.0};
    Vector2 pA;
    Vector2 pB;
    Vector2 pC;

    Triangle() = default;
    explicit Triangle(Vector2 a, Vector2 b, Vector2 c) : pA(a), pB(b), pC(c) {
        acL = Line2(a,c);
        abL = Line2(a,b);
        fB = acL(b);
        fC = abL(c);
    }
    
    Vector3 operator()(const Vector2& p) {
        Float b = acL(p) / fB;
        Float c = abL(p) / fC;
        Float a = 1.0 - b - c;
        return Vector3(a,b,c);
    }
    
    bool test(const Vector2& p) {
        const Vector3 bc = (*this)(p);
        return inRange(bc.x(), 0.0, 1.0) && inRange(bc.y(), 0.0, 1.0) && inRange(bc.z(), 0.0, 1.0);
    }
};

