#pragma once
#include <soft3d/common/util.h>
#include <soft3d/image/image.h>
#include <soft3d/math/linalg.h>

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

    Float operator()(const Vector2& p) const {
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

    Vector3 operator()(const Vector2& p) const {
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
        n = ab.cross(ac);
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
        a = clampToNormal(a);
        b = clampToNormal(b);
        c = clampToNormal(c);
        return Vector3(a, b, c);
    }

    bool sameFace(const Vector3& v) const {
        Vector3 ab = pB - pA;
        Vector3 ac = pC - pA;
        return v.dot(ab.cross(ac)) >= 0.0f;
    }

    Triangle3 transformed(const Matrix& mat, Vector3& homo) const {
        Vector4 a = mat * pA.expand(1.0f);
        Vector4 b = mat * pB.expand(1.0f);
        Vector4 c = mat * pC.expand(1.0f);
        homo = Vector3(a.w(), b.w(), c.w());
        return Triangle3(a.homoDiv(), b.homoDiv(), c.homoDiv());
    }
};

// We define parametric function for points
// parametrization = f(x) -> f(g(u))
// usually "reparametrize" by arc length
// arclength of parametric function is s = \int ||f'||dt
// arc length parametization when |df(s)/ds| = c
// in this mode, we should be able t solve t given s
// then f(t) = f(t(s))
// u = free paramter s = arc length parameter
//
// canonical form:
// p = \sum c_i b_i(t)
// in polynomial b_i = t^i
// It's linear comination of b_i
// we set up linear system by propeties at the endpoints
// a = c here
// e.g. line
// p_0 = [1 0] a_0
// p_1 = [1 1] a_1
// we call coeff matrix C (constraint matrix)
// p = Ca
// then
// f(u) = u B p
// where B is C^-1 (basis matrix)
struct Spline {
    virtual ~Spline() {
    }
    virtual Vector2 sample(Float s) = 0;
    void draw(Image& screen, Vector3 start, Vector3 end, Float step) {
        for (Float s = 0; s < 1.0f; s += step) {
            Vector2 pos = sample(s);
            Vector3 color = start * (1 - s) + s * end;
            screen.setPixel(pos, color);
        }
    }
};

struct KnotControl {
    Float x;
    Float y;
    Float knot;
};

// f_i(u) = (1-u)p_1+up_2
struct KnotSpline : public Spline {
    std::vector<KnotControl> controls;
    KnotSpline() {
    }
    ~KnotSpline() {
    }

    Vector2 sample(Float s) override {
        for (int i = controls.size() - 2; i >= 0; --i) {
            if (s > controls[i].knot) {
                Float u = (s - controls[i].knot) / (controls[i + 1].knot - controls[i].knot);
                Vector2 p1(controls[i].x, controls[i].y);
                Vector2 p2(controls[i + 1].x, controls[i + 1].y);
                return (1 - u) * p1 + u * p2;
            }
        }
        return Vector2(0, 0);
    }
};

struct CubicControl {
    Vector2 pos;
    Vector2 d;
    Vector2 dd;
    Float knot;
};

// f(x) = canonical
// find derivate and substitute
// f(0) = a_0
// f'(0) = a_1
// f''(0) = 2 * a_2
// f(1) = a_0 + a_1 + a_2 + a_3
// C = [1,0,0,0 ; 0,1,0,0 ; 0,0,2,0 ; 1,1,1,1]
// B = [1,0,0,0 ; 0,1,0,0 ; 0,0,0.5,0 ; -1,-1,-0.5,1]
struct CubicSpline : public Spline {
    Matrix basis{ Matrix(4, 4,
                         { 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.5f, 0.0f,
                           -1.0f, -1.0f, -0.5f, 1.0f }) };
    std::vector<CubicControl> controls;
    CubicSpline() = default;
    ~CubicSpline() {
    }

    Vector2 sample(Float s) override {
        for (int i = controls.size() - 2; i >= 0; --i) {
            if (s > controls[i].knot) {
                Float u = (s - controls[i].knot) / (controls[i + 1].knot - controls[i].knot);
                Vector4 uVec = Vector4(1.0, u, u * u, pow(u, 3.0));
                Vector4 ux(controls[i].pos.x(), controls[i].d.x(), controls[i].dd.x(),
                           controls[i + 1].pos.x());
                Vector4 uy(controls[i].pos.y(), controls[i].d.y(), controls[i].dd.y(),
                           controls[i + 1].pos.y());
                Float x = uVec.dot(basis * ux);
                Float y = uVec.dot(basis * uy);
                return Vector2(x, y);
            }
        }
        return Vector2(0, 0);
    }
};

struct CardinalControl {
    Float x;
    Float y;
};

// Connect p_2 and p_3 by calculated derivative from p_1p_3 p_2p_4
// t is tension parameter
// it scales the derivatives
// f(0) = p2
// f(1) = p_3
// f'(0) = 1/2(1-t)(p_3-p_1)
// f'(1) = 1/2(1-t)(p_4-p_2)
// p_0 = f(1) - 2/(1-t)f'(0)
// p_1 = f(0)
// p_2 = f(1)
// p_3 = f(0) + 2/(1-t)f'(1)
// Solving constraint matrix from this system yields
// B = [ 0,1,0,0 ; -s,0,s,0 ; 2s,s-3,3-2s,-s ; -s,2-s,s-2,s ]
// s = (1-t)/2
struct CardinalSpline : public Spline {
    std::vector<CardinalControl> controls;
    CardinalSpline() = default;
    CardinalSpline(Float tension) {
        s = (1.0f - tension) / 2.0f;
        basis = Matrix(
            4, 4, { 0, 1, 0, 0, -s, 0, s, 0, 2 * s, s - 3, 3 - 2 * s, -s, -s, 2 - s, s - 2, s });
    }
    ~CardinalSpline() {
    }

    Vector2 sample(Float s) override {
        int n = controls.size() - 3;
        int i = n * s;
        Float u = n * s - i;
        Vector4 uVec = Vector4(1.0, u, u * u, pow(u, 3.0));
        Vector4 ux(controls[i].x, controls[i + 1].x, controls[i + 2].x, controls[i + 3].x);
        Vector4 uy(controls[i].y, controls[i + 1].y, controls[i + 2].y, controls[i + 3].y);
        Float x = uVec.dot(basis * ux);
        Float y = uVec.dot(basis * uy);
        return Vector2(x, y);
    }

  private:
    Float s;
    Matrix basis;
};
