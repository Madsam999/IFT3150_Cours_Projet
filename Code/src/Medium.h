//
// Created by Samuel on 2024-09-21.
//

#pragma once

#include "linalg/linalg.h"
#include "basic.h"
#include "aabb.h"
#include "object.h"
#include "pcg32/pcg32.h"

using namespace linalg::aliases;

#define DDA 0
#define RayMarching 1

class Voxel {
public:
    Voxel() {};
    Voxel(double density): density(density) {}

    double density;
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
    Medium(int voxel_x, int voxel_y, int voxel_z, std::string traversalType) {
        this->voxelCounts = int3(voxel_x, voxel_y, voxel_z);
        this->voxels = std::vector<Voxel>(voxelCounts.x * voxelCounts.y * voxelCounts.z);
        createVoxels();
        this->voxelSize = double3(1.0 / voxel_x, 1.0 / voxel_y, 1.0 / voxel_z);
        // pink rgb between 0 and 1
        srand (time(NULL));
        this->mediumColor = double3(0.18, 0.812, 0.569);
        this->traversalType = traversalType == "DDA" ? DDA : RayMarching;
        this->scatter = double3(0.1, 0.1, 0.1);
    };

    bool traversalType;

    int3 voxelCounts;

    std::vector<Voxel> voxels;

    double3 minBound = double3(0,0,0);
    double3 maxBound = double3(1,1,1);

    double3 voxelSize;

    double3 mediumColor;

    double3 scatter;

    bool intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
        Ray lray{mul(i_transform, {ray.origin,1}).xyz(), mul(i_transform, {ray.direction,0}).xyz()};
        return local_intersect(lray, t_min, t_max, hit);
    }
    
    bool local_intersect(Ray ray, double t_min, double t_max, Intersection *hit);

    void createVoxels() {
        for(int x = 0; x < voxelCounts.x; x++) {
            for (int y = 0; y < voxelCounts.y; y++) {
                for (int z = 0; z < voxelCounts.z; z++) {
                    voxels[x + y * voxelCounts.x + z * voxelCounts.x * voxelCounts.y] = Voxel(rand_double());
                }
            }
        }
    }

    void DDA_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    void RayMarching_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {
        return;
    }

    double3 transferFunction_color(double density) const {
        return double3(density * mediumColor.x, density * mediumColor.y, density * mediumColor.z);
    }

    double transferFunction_opacity(double density) {
        return density * 0.1;
    }

    double triLinearInterpolation(double3 position, int3 voxelPosition);
};


