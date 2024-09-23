//
// Created by Samuel on 2024-09-21.
//

#pragma once

#include "linalg/linalg.h"
#include "basic.h"
#include "aabb.h"
#include "object.h"

using namespace linalg::aliases;

class Medium {
public:

    double4x4 transform;
    double4x4 i_transform;

    void setup_transform(double4x4 m)
    {
        transform = m;
        i_transform = inverse(m);
    };

    Medium() {};
    Medium(int voxel_x, int voxel_y, int voxel_z, float voxelSize) {
        this->voxel_x = voxel_x;
        this->voxel_y = voxel_y;
        this->voxel_z = voxel_z;
        this->voxelSize = voxelSize;
    };

    int voxel_x;
    int voxel_y;
    int voxel_z;

    float voxelSize;

    double3 position;

    bool intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
        Ray lray{mul(i_transform, {ray.origin,1}).xyz(), mul(i_transform, {ray.direction,0}).xyz()};
        //std::cout << "lego" << std::endl;
        if(local_intersect(lray, t_min, t_max, hit)){
            //std::cout << "HIT" << std::endl;
            hit->position = mul(transform, {hit->position, 1}).xyz();
            hit->normal = normalize(mul(transform, {hit->normal, 0}).xyz());
            return true;
        }
        return false;
    }

    bool local_intersect(Ray ray, double t_min, double t_max, Intersection *hit);

};

class Voxel {
public:
    Voxel() {};
    Voxel(float density, int3 position) {};

    float density;
    int3 position;
};