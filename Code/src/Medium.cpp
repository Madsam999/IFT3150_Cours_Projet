//
// Created by Samuel on 2024-09-21.
//

#include "Medium.h"
#include <fstream>  // Include this for file handling

bool Medium::local_intersect(Ray ray, double t_min, double t_max, Intersection *hit) {

    /*
     * Compute intersection with cube.
     * Bottom back left corner is at (0,0,0)
     * Top front right corner is at (1,1,1)
     */

    // Inverse ray direction to avoid division by zero
    double3 inv_dir = {1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z};

    double3 t0 = (minBound - ray.origin) * inv_dir;
    double3 t1 = (maxBound - ray.origin) * inv_dir;

    double3 tmin = min(t0, t1);
    double3 tmax = max(t0, t1);

    double tmin_max = std::max(tmin.x, std::max(tmin.y, tmin.z));
    double tmax_min = std::min(tmax.x, std::min(tmax.y, tmax.z));

    if (tmax_min < tmin_max) {
        return false;
    }

    double t = tmin_max;

    if (t < t_min || t > t_max) {
        return false;
    }

    // std::cout << "Tmin: " << tmin_max << " Tmax: " << tmax_min << std::endl;

    double3 start = ray.origin + ray.direction * tmin_max;
    double3 end = ray.origin + ray.direction * tmax_min;

    hit->position = start;
    hit->depth = tmin_max;

    auto tMin = tmin_max;
    auto tMax = tmax_min;

    return DDA(start, end, hit, ray, tMin, tMax);
}

bool Medium::DDA(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {

    double3 test = ray.origin + ray.direction * 50;

    // std::cout << "Test: " << test.x << " " << test.y << " " << test.z << std::endl;

    int3 entryVoxels, exitVoxels;
    if(start.x < 1.0) {
        entryVoxels.x = start.x * voxel_x;
    }
    else {
        entryVoxels.x = voxel_x - 1;
    }
    if(start.y < 1.0) {
        entryVoxels.y = start.y * voxel_y;
    }
    else {
        entryVoxels.y = voxel_y - 1;
    }
    if(start.z < 1.0) {
        entryVoxels.z = start.z * voxel_z;
    }
    else {
        entryVoxels.z = voxel_z - 1;
    }



    if(end.x < 1.0) {
        exitVoxels.x = end.x * voxel_x;
    }
    else {
        exitVoxels.x = voxel_x - 1;
    }
    if(end.y < 1.0) {
        exitVoxels.y = end.y * voxel_y;
    }
    else {
        exitVoxels.y = voxel_y - 1;
    }
    if(end.z < 1.0) {
        exitVoxels.z = end.z * voxel_z;
    }
    else {
        exitVoxels.z = voxel_z - 1;
    }

    if((entryVoxels == int3(0,0,1)) && (exitVoxels == int3(0, 0,0))) {
        // std::cout << "EntryVoxels: " << entryVoxels.x << " " << entryVoxels.y << " " << entryVoxels.z << std::endl;
    }

    if(voxels[entryVoxels.x + entryVoxels.y * voxel_x + entryVoxels.z * voxel_x * voxel_y].density > 0) {
        // hit->position = {entryVoxels.x * voxelSize.x, entryVoxels.y * voxelSize.y, entryVoxels.z * voxelSize.z};
        return true;
    }



    while(entryVoxels.x != exitVoxels.x || entryVoxels.y != exitVoxels.y || entryVoxels.z != exitVoxels.z) {
        double3 voxelPosition = double3(entryVoxels.x * voxelSize.x, entryVoxels.y * voxelSize.y, entryVoxels.z * voxelSize.z);

        double3 nextPlanes;
        double3 tDeltas;

        if(ray.direction.x > 0) {
            nextPlanes.x = (entryVoxels.x + 1) * voxelSize.x;
            tDeltas.x = voxelSize.x / ray.direction.x;
        }
        else if(ray.direction.x < 0) {
            nextPlanes.x = entryVoxels.x * voxelSize.x;
            tDeltas.x = -voxelSize.x / ray.direction.x;
        }
        else {
            nextPlanes.x = DBL_MAX;
            tDeltas.x = DBL_MAX;
        }
        if(ray.direction.y > 0) {
            nextPlanes.y = (entryVoxels.y + 1) * voxelSize.y;
            tDeltas.y = voxelSize.y / ray.direction.y;
        }
        else if(ray.direction.y < 0) {
            nextPlanes.y = entryVoxels.y * voxelSize.y;
            tDeltas.y = -voxelSize.y / ray.direction.y;
        }
        else {
            nextPlanes.y = DBL_MAX;
            tDeltas.y = DBL_MAX;
        }
        if(ray.direction.z > 0) {
            nextPlanes.z = (entryVoxels.z + 1) * voxelSize.z;
            tDeltas.z = voxelSize.z / ray.direction.z;
        }
        else if(ray.direction.z < 0) {
            nextPlanes.z = entryVoxels.z * voxelSize.z;
            tDeltas.z = -voxelSize.z / ray.direction.z;
        }
        else {
            nextPlanes.z = DBL_MAX;
            tDeltas.z = DBL_MAX;
        }
        double3 tMaxes;
        tMaxes.x = (nextPlanes.x - ray.origin.x) / ray.direction.x;
        tMaxes.y = (nextPlanes.y - ray.origin.y) / ray.direction.y;
        tMaxes.z = (nextPlanes.z - ray.origin.z) / ray.direction.z;

        double updateTMin = std::min(tMaxes.x, std::min(tMaxes.y, tMaxes.z));

        tMin = updateTMin;

        double3 newStart = ray.origin + ray.direction * tMin;

        if(ray.direction.x > 0) {
            entryVoxels.x = newStart.x < 1 ? std::floor(newStart.x * voxel_x) : voxel_x - 1;
        }
        else if(ray.direction.x < 0) {
            entryVoxels.x = newStart.x < 1 ? std::ceil(newStart.x * voxel_x) - 1 : 0;
        }
        if(ray.direction.y > 0) {
            entryVoxels.y = newStart.y < 1 ? std::floor(newStart.y * voxel_y) : voxel_y - 1;
        }
        else if(ray.direction.y < 0) {
            entryVoxels.y = newStart.y < 1 ? std::ceil(newStart.y * voxel_y) - 1 : 0;
        }
        if(ray.direction.z > 0) {
            entryVoxels.z = newStart.z < 1 ? std::floor(newStart.z * voxel_z) : voxel_z - 1;
        }
        else if(ray.direction.z < 0) {
            entryVoxels.z = newStart.z < 1 ? std::ceil(newStart.z * voxel_z) - 1 : 0;
        }
        if(voxels[entryVoxels.x + entryVoxels.y * voxel_x + entryVoxels.z * voxel_x * voxel_y].density > 0) {
            hit->position = {entryVoxels.x * voxelSize.x, entryVoxels.y * voxelSize.y, entryVoxels.z * voxelSize.z};
            return true;
        }

    }
    return false;
}