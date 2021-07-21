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
    Float minX{ 0.0f };
    Float minY{ 0.0f };
    Float minZ{ 0.0f };
    Float maxX{ 0.0f };
    Float maxY{ 0.0f };
    Float maxZ{ 0.0f };

    BoundingRect operator+(const BoundingRect& other) const {
        BoundingRect rect { std::min(minX, other.minX), std::min(minY, other.minY),
                             std::min(minZ, other.minZ), std::max(maxX, other.maxX),
                             std::max(maxY, other.maxY), std::max(maxZ, other.maxZ) };
        return rect;
    }

    Float minComp(int axis) const {
        if (axis == 0) {
            return minX;
        } else if (axis == 1) {
            return minY;
        } else {
            return minZ;
        }
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
    bool hit(const Ray& ray) const {
        Float facx = 1.0 / ray.dir.x();
        Float tminX = (minX - ray.origin.x()) * facx;
        Float tmaxX = (maxX - ray.origin.x()) * facx;
        if (ray.dir.x() < 0) {
            std::swap(tminX, tmaxX);
        }
        Float facy = 1.0 / ray.dir.y();
        Float tminY = (minY - ray.origin.y()) * facy;
        Float tmaxY = (maxY - ray.origin.y()) * facy;
        if (ray.dir.y() < 0) {
            std::swap(tminY, tmaxY);
        }
        if (tminX > tmaxY || tmaxX < tminY) {
            return false;
        }
        Float facz = 1.0 / ray.dir.z();
        Float tminZ = (minZ - ray.origin.z()) * facz;
        Float tmaxZ = (maxZ - ray.origin.z()) * facz;
        if (ray.dir.z() < 0) {
            std::swap(tminZ, tmaxZ);
        }
        if (tminX > tmaxZ || tmaxX < tminZ) {
            return false;
        } else {
            return true;
        }
    }

    Vector3 mid() const {
        return Vector3((minX + maxX) / 2.0f, (minY + maxY) / 2.0f, (minZ + maxZ) / 2.0f);
    }
};
