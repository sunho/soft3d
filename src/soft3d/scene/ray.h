#pragma once

struct Geometry;

struct Ray {
    Vector3 origin;
    Vector3 dir;
    bool isShadow{ false };
};

struct RayHit {
    Vector3 normal;
    Vector3 pos;
    Float time;
    const Geometry* geom;
};

struct BoundingRect {
    Vector3 min{};
    Vector3 max{};

    BoundingRect operator+(const BoundingRect& other) const {
        BoundingRect rect { Vector3(std::min(min[0], other.min[0]), std::min(min[1], other.min[1]),
                             std::min(min[2], other.min[2])), Vector3(std::max(max[0], other.max[0]),
                             std::max(max[1], other.max[1]), std::max(max[2], other.max[2])) };
        
        return rect;
    }

    Float minComp(int axis) const {
        return min[axis];
    }

    // Assuming ray.dir.x y z >= 1
    // t_xmin = x_min - x_e / x_d
    // ...
    // the ray hits iff [t_xmin, t_xmax], [t_ymin, t_ymax], and [t_zmin, y_zmax] overlaps
    // for x_d < 0, t_xmin and t_xmax flip
    // fox x_d = 0 it's more subtle
    // ray hit iff x_min < x_e < x_max
    // no hit when x_e <= x_min
    // or x_e >= x_max
    // for first case: t_max = inf t_min = inf (inf,inf) -> no hit
    // for hit case: t_min = -inf t_max = inf (-inf,inf) -> every hit
    // for NaN -> conditions will be evaluated as false <- this case is just taken as hit actually
    // as IEEE rule
    // ---
    // fast 
    // clip tims intervals by each parallel plane
    // when we merge each clipped plane we do max(start,start), min(end,end)
    // note that when they are not overlapping 
    // start and end point get "reversed"
    inline bool hit(const Ray& ray, const Vector3& invDir) const {
        // This is slower somehow
        /*Vector3 tt = min;
        tt -= ray.origin;
        tt /= ray.dir;
        Vector3 tt2 = max;
        tt2 -= ray.origin;
        tt2 /= ray.dir;
        Vector3 tmin = tt.min(tt2);
        Vector3 tmax = tt.max(tt2);
        Float clipmin = tmin.hmax();
        Float clipmax = tmax.hmin();
        if (clipmax < 0.0f || clipmin > clipmax)
            return false;
        return true;*/
        Float tminX = (min[0] - ray.origin.x()) * invDir.x();
        Float tmaxX = (max[0] - ray.origin.x()) * invDir.x();
        if (ray.dir.x() < 0) {
            std::swap(tminX, tmaxX);
        }
        Float tminY = (min[1] - ray.origin.y()) * invDir.y();
        Float tmaxY = (max[1] - ray.origin.y()) * invDir.y();
        if (ray.dir.y() < 0) {
            std::swap(tminY, tmaxY);
        }
        if (tminX > tmaxY || tmaxX < tminY) {
            return false;
        }
        Float tminZ = (min[2] - ray.origin.z()) * invDir.z();
        Float tmaxZ = (max[2] - ray.origin.z()) * invDir.z();
        if (ray.dir.z() < 0) {
            std::swap(tminZ, tmaxZ);
        }
        if (tminX > tmaxZ || tmaxX < tminZ || tminY > tmaxZ || tminZ > tmaxY) {
            return false;
        } else {
            return true;
        }
    }

    Vector3 mid() const {
        return (min + max) / 2.0f;
    }
};
