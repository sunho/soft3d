#pragma once
#include <nuguri3d/common/image.h>
#include <nuguri3d/common/linalg.h>
#include <nuguri3d/common/util.h>

#include <cmath>

using DrawFunc = std::function<void(Vector2 pos, Vector3 color)>;

// f(x,y) = Ax + By + C
// f(x,y) = (y0 - y1)x + (x1 - x0)y + x0y1 - x1y0 = 0
// distance = k sqrt(A^2 + B^2) = f(x,y) / sqrt(A^2 + B^2)
struct Line2 {
    Vector2 a;
    Vector2 b;
    Vector2 AB{};
    Float C{ 0.0 };
    Float slope;
    Float normalizer{ 0.0 };

    Line2() = default;
    explicit Line2(const Vector2& a, const Vector2& b) : a(a), b(b) {
        AB = Vector2(a.y() - b.y(), b.x() - a.x());
        C = a.x() * b.y() - b.x() * a.y();
        normalizer = sqrt(a.dot(a) + b.dot(b));
        slope = -AB.x() / AB.y();
    }

    Float operator()(const Vector2& p) {
        return AB.dot(p) + C;
    }

    Float distance(const Vector2& p) {
        return (*this)(p) / normalizer;
    }

    void draw(Image& screen, const Vector3& color) {
        Vector2 px0 = a.x() < b.x() ? a : b;
        Vector2 px1 = a.x() < b.x() ? b : a;
        Vector2 py0 = a.y() < b.y() ? a : b;
        Vector2 py1 = a.y() < b.y() ? b : a;
        if (slope > 1.0) {
            int x = py0.x();
            for (int y = py0.y(); y < py1.y(); ++y) {
                screen.setPixel(x, y, color);
                if ((*this)(Vector2(x + 0.5, y + 1)) > 0) {
                    ++x;
                }
            }
        } else if (slope > 0.0) {
            int y = px0.y();
            for (int x = px0.x(); x < px1.x(); ++x) {
                screen.setPixel(x, y, color);
                if ((*this)(Vector2(x + 1, y + 0.5)) < 0) {
                    ++y;
                }
            }
        } else if (slope > -1.0) {
            int y = px0.y();
            for (int x = px0.x(); x < px1.x(); ++x) {
                screen.setPixel(x, y, color);
                if ((*this)(Vector2(x + 1, y - 0.5)) > 0) {
                    --y;
                }
            }
        } else {
            int x = py0.x();
            for (int y = py0.y(); y < py1.y(); ++y) {
                screen.setPixel(x, y, color);
                if ((*this)(Vector2(x + 0.5, y + 1)) > 0) {
                    --x;
                }
            }
        }
    }
};

// barycentric beta is the signed scaled distance from the l_AC
// beta = f_ac (x, y) / f_ac(x_b, y_b)
// f_ac(x_b,y_b) is the area
struct Triangle2 {
    Line2 acL;
    Line2 abL;
    Float fB{ 0.0 };
    Float fC{ 0.0 };
    Vector2 pA;
    Vector2 pB;
    Vector2 pC;

    Triangle2() = default;
    explicit Triangle2(Vector2 a, Vector2 b, Vector2 c) : pA(a), pB(b), pC(c) {
        acL = Line2(a, c);
        abL = Line2(a, b);
        fB = acL(b);
        fC = abL(c);
    }

    Vector3 operator()(const Vector2& p) {
        Float b = acL(p) / fB;
        Float c = abL(p) / fC;
        Float a = 1.0 - b - c;
        return Vector3(a, b, c);
    }

    bool test(const Vector2& p) {
        const Vector3 bc = (*this)(p);
        return inRange(bc.x(), 0.0, 1.0) && inRange(bc.y(), 0.0, 1.0) && inRange(bc.z(), 0.0, 1.0);
    }

    void draw(Image& screen, const Vector3& cA, const Vector3& cB, const Vector3& cC) {
        for (int i = 0; i < screen.getWidth(); ++i) {
            for (int j = 0; j < screen.getHeight(); ++j) {
                Vector3 bary = (*this)(Vector2(i, j));
                if (nearInRange(bary.x(), 0.0, 1.0) && nearInRange(bary.y(), 0.0, 1.0) &&
                    nearInRange(bary.z(), 0.0, 1.0)) {
                    Vector3 color = bary.x() * cA + bary.y() * cB + bary.z() * cC;
                    screen.setPixel(i, j, color);
                }
            }
        }
    }
};

struct Triangle3 {
    Vector3 pA;
    Vector3 pB;
    Vector3 pC;
    Vector3 n;
    Vector3 cA;
    Vector3 cB;
    Vector3 cC;

    Triangle3() = default;
    explicit Triangle3(Vector3 a, Vector3 b, Vector3 c) : pA(a), pB(b), pC(c) {
        Vector3 ab = pB - pA;
        Vector3 ac = pC - pA;
        n = ab.cross(ac).normalized();
        cA = pC - pB;
        cB = pA - pC;
        cC = pB - pA;
    }
    // The barycentric coordinates are proportional to the areas of the three subtriangles.
    // a = A_a / A
    // A_a = |(c-b)x(p - b)| / 2
    // A = |(b-a)x(c-a)| / 2
    // (b-a)x(c-a) = normal (it's perpendicular to the surface)
    // Note that signity of bc x bp and ab x ac is is same when p is inside the triangle
    // Use dot product to test this and normalize it cleverly
    // a = n.(c-b)x(p - b) / |n|^2
    // = |n| * |(c-b)x(p-b)| * cos(theta) / |n|^2
    // = |(c-b)x(p-b)| / |n|
    // = A_a / A
    // cos(theta) = 1 when it's inside the triangle
    // Resulting formulas
    // n = (b-a)x(c-a)
    // n_a = (c-b)x(p-b)
    // n_b = (a-c)x(p-c)
    // n_c = (b-a)x(p-a)
    // a = n.n_a/|n|^2
    // b = n.n_b/|n|^2
    // c = n.n_c/|n|^2
    Vector3 operator()(const Vector3& p) const {
        Vector3 nA = cA.cross(p - pB);
        Vector3 nB = cB.cross(p - pC);
        Vector3 nC = cC.cross(p - pA);
        Float n2 = n.norm2();
        Float a = n.dot(nA) / n2;
        Float b = n.dot(nB) / n2;
        Float c = n.dot(nC) / n2;
        return Vector3(a, b, c);
    }

    Triangle3 transformed(const Matrix& mat) const {
        return Triangle3(pA.transformed(mat, 1.0), pB.transformed(mat, 1.0),
                         pC.transformed(mat, 1.0));
    }
};
