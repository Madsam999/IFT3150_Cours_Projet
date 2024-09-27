//
// Created by Samuel on 2024-09-21.
//

#include "Medium.h"

bool Medium::local_intersect(Ray ray, double t_min, double t_max, Intersection *hit) {

    /*
     * Compute intersection with cube.
     * Bottom back left corner is at (0,0,0)
     * Top front right corner is at (1,1,1)
     */

    // std::cout << hit->position.x << hit->position.y << hit->position.z << std::endl;

    double3 inv_dir = {1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z};

    double3 t0 = (double3(0,0,0) - ray.origin) * inv_dir;
    double3 t1 = (double3(1,1,1) - ray.origin) * inv_dir;

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

    double3 start = ray.origin + ray.direction * tmin_max;
    double3 end = ray.origin + ray.direction * tmax_min;


    bool test = DDA(start, end, hit);

    return true;
}

bool Medium::DDA(double3 start, double3 end, Intersection *hit) {
    //std::cout << "x:" << start.x << "y:" << start.y << "z:" << start.z << std::endl;

    float sizeOfVoxelX = 1.0/voxel_x;
    float sizeOfVoxelY = 1.0/voxel_y;
    float sizeOfVoxelZ = 1.0/voxel_z;

    int startVoxelX = start.x < 1 ? std::floor(start.x / sizeOfVoxelX) : voxel_x - 1;
    // Inverse le y, car la boite est construit du bas vers le haut, mais le raytracer scan du haut vers le bas
    int startVoxelY = start.y < 1 ? 1 - std::floor(start.y / sizeOfVoxelY) : voxel_y - 1;
    int startVoxelZ = start.z < 1 ? std::floor(start.z / sizeOfVoxelZ) : voxel_z - 1;

    int endVoxelX = end.x < 1 ? std::floor(end.x / sizeOfVoxelX) : voxel_x - 1;
    // Inverse le y, car la boite est construit du bas vers le haut, mais le raytracer scan du haut vers le bas
    int endVoxelY = end.y < 1 ? 1 - std::floor(end.y / sizeOfVoxelY) : voxel_y - 1;
    int endVoxelZ = end.z < 1 ? std::floor(end.z / sizeOfVoxelZ) : voxel_z - 1;

    if(startVoxelX < 0 || startVoxelX >= voxel_x || startVoxelY < 0 || startVoxelY >= voxel_y || startVoxelZ < 0 || startVoxelZ >= voxel_z) {
        return false;
    }

    float stepX = startVoxelX < endVoxelX ? 1.0f/voxel_x : -1.0f/voxel_x;
    float stepY = startVoxelY < endVoxelY ? 1.0f/voxel_y : -1.0f/voxel_y;
    float stepZ = startVoxelZ < endVoxelZ ? 1.0f/voxel_z : -1.0f/voxel_z;

    float planeToHitX = startVoxelX + stepX;
    float planeToHitY = startVoxelY + stepY;
    float planeToHitZ = startVoxelZ + stepZ;

#

    return true;
}
