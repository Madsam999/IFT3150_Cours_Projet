//
// Created by Samuel on 2024-09-21.
//

#pragma once

#include "linalg/linalg.h"
#include "basic.h"
#include "aabb.h"
#include "object.h"

using namespace linalg::aliases;



class Voxel {
public:
    Voxel() {};
    Voxel(float density, double3 position) {
        this->density = density;
        this->position = position;
    }

    float density;
    double3 position;
};

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
        this->voxels = std::vector<Voxel>(voxel_x * voxel_y * voxel_z);
        createVoxels();
    };

    int voxel_x;
    int voxel_y;
    int voxel_z;

    std::vector<double3> bounds;

    float voxelSize;

    std::vector<Voxel> voxels;

    double3 position;

    bool intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
        Ray lray{mul(i_transform, {ray.origin,1}).xyz(), mul(i_transform, {ray.direction,0}).xyz()};
        if(local_intersect(lray, t_min, t_max, hit)){
            hit->position = mul(transform, {hit->position, 1}).xyz();
            hit->normal = normalize(mul(transform, {hit->normal, 0}).xyz());
            return true;
        }
        return false;
    }
    
    bool local_intersect(Ray ray, double t_min, double t_max, Intersection *hit);

    bool intersectVoxels(double3 start, double3 end, Intersection *hit);

    std::vector<Voxel> createVoxels() {
        for(int x = 0; x < voxel_x; x++) {
            for(int y = 0; y < voxel_y; y++) {
                for(int z = 0; z < voxel_z; z++) {
                    double3 position = double3((x - (voxel_x/2)) * voxelSize, (y - (voxel_y/2)) * voxelSize, (z - (voxel_z/2)) * voxelSize);
                    voxels[x + y * voxel_x + z * voxel_x * voxel_y] = Voxel(0, position);
                }
            }
        }

        voxels[3 + 3*voxel_x + 3*voxel_x*voxel_y].density = 1;

        return voxels;
    }
};

