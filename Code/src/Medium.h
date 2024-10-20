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

class Scene;

class Voxel {
public:
    Voxel() {};
    Voxel(double density);

    double density;
};

class Medium {
public:
    void setup_transform(double4x4 m);
    Medium() {};
    Medium(int voxel_x, int voxel_y, int voxel_z, int traversalType);
    bool intersect(Ray ray, double t_min, double t_max, Intersection *hit);
    bool local_intersect(Ray ray, double t_min, double t_max, Intersection *hit);
    void createVoxels();
    void DDA_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    void RayMarching_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    double3 transferFunction_color(double density) const;
    double transferFunction_opacity(double density) const;
    double triLinearInterpolation(double3 position, int3 voxelPosition);
    bool testLightIntersection(Ray ray, double t_min, double* t_max);

    enum TraversalType {
        DDA = 0,
        RegularStepRM = 1,
        RegularStepJitterRM = 2,
        MiddleRayVoxelRM = 3,
        MiddleRayVoxelJitterRM = 4
    };

    float stepSize;

    TraversalType traversalType;

    int3 voxelCounts;

    std::vector<Voxel> voxels;

    double3 minBound = double3(0,0,0);
    double3 maxBound = double3(1,1,1);

    double3 voxelSize;

    double3 mediumColor;

    double4x4 transform;
    double4x4 i_transform;

    double sigma_a = 0.1;

    Scene* scene;
};



