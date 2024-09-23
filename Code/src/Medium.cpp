//
// Created by Samuel on 2024-09-21.
//

#include "Medium.h"

bool Medium::local_intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
    // Compute the intersection with the medium
    double tx1, tx2, ty1, ty2, tz1, tz2;
    double tmin, tmax;
    auto xMin = -1 * (voxel_x * voxelSize) / 2;
    auto xMax = (voxel_x * voxelSize) / 2;
    auto yMin = -1 * (voxel_y * voxelSize) / 2;
    auto yMax = (voxel_y * voxelSize) / 2;
    auto zMin = -1 * (voxel_z * voxelSize) / 2;
    auto zMax = (voxel_z * voxelSize) / 2;

    tx1 = (xMin - ray.origin.x) / ray.direction.x;
    tx2 = (xMax - ray.origin.x) / ray.direction.x;
    ty1 = (yMin - ray.origin.y) / ray.direction.y;
    ty2 = (yMax - ray.origin.y) / ray.direction.y;
    tz1 = (zMin - ray.origin.z) / ray.direction.z;
    tz2 = (zMax - ray.origin.z) / ray.direction.z;

    tmin = std::max(std::min(tx1, tx2), std::max(std::min(ty1, ty2), std::min(tz1, tz2)));
    tmax = std::min(std::max(tx1, tx2), std::min(std::max(ty1, ty2), std::max(tz1, tz2)));

    if (tmin > tmax || tmax < 0) {
        return false;
    }

    hit->depth = tmin;
    double3 p = ray.origin + tmin * ray.direction;
    hit->position = p;

    return true;
}
