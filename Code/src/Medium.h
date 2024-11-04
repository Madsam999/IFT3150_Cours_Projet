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
    /*
     * @brief Implémentations des fonctions de base de la classe.
     */
    Medium() {};
    /**
     * @brief Set the transform object
     *
     * @param m
     */
    void setup_transform(double4x4 m) {
        this->transform = m;
        this->i_transform = inverse(m);
    }
    /**
     * @brief Construct a new Medium object
     *
     * @param voxel_x
     * @param voxel_y
     * @param voxel_z
     * @param traversalType
     */
    Medium(int voxel_x, int voxel_y, int voxel_z, int traversalType) {
        this->voxelCounts = int3(voxel_x, voxel_y, voxel_z);
        this->voxels = std::vector<Voxel>(voxelCounts.x * voxelCounts.y * voxelCounts.z);
        this->voxelSize = double3(1.0 / voxel_x, 1.0 / voxel_y, 1.0 / voxel_z);
        this->mediumColor = double3(62.0, 163.0, 194.0) / 255.0;
        switch (traversalType) {
            case 0:
                this->traversalType = DDA;
                break;
            case 1:
                this->traversalType = RegularStepRM;
                break;
            case 2:
                this->traversalType = RegularStepJitterRM;
                break;
            case 3:
                this->traversalType = MiddleRayVoxelRM;
                break;
            case 4:
                this->traversalType = MiddleRayVoxelJitterRM;
                break;
        }
        createVoxels();
        this->stepSize = 0.005;
    }
    /**
     *
     * @param ray
     * @param t_min
     * @param t_max
     * @param hit
     * @return
     */
    bool intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
        Ray lray{mul(i_transform, {ray.origin,1}).xyz(), mul(i_transform, {ray.direction,0}).xyz()};
        return local_intersect(lray, t_min, t_max, hit);
    }
    /**
     *
     * @param density
     * @return
     */
    void createVoxels() {
        for(int x = 0; x < voxelCounts.x; x++) {
            for (int y = 0; y < voxelCounts.y; y++) {
                for (int z = 0; z < voxelCounts.z; z++) {
                    double density = rand_double();
                    this->voxels[x + y * voxelCounts.x + z * voxelCounts.x * voxelCounts.y] = Voxel(density);
                }
            }
        }
    }
    /**
     *
     * @param density
     * @return
     */
    double3 transferFunction_color(double density) const {
        return mediumColor;
    }
    /**
     *
     * @param density
     * @return
     */
    double transferFunction_opacity(double density) const {
        return 1 - std::exp(-density * stepSize);
    }

    /*
     * @brief Fonctions qui seront implémentés dans Medium.cpp.
     */
    bool local_intersect(Ray ray, double t_min, double t_max, Intersection *hit);
    void DDA_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    void RayMarching_Traversal_Regular_Step(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    void RayMarching_Traversal_Regular_Step_Jitter(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    void RayMarching_Traversal_Middle_Ray_Voxel(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    void RayMarching_Traversal_Middle_Ray_Voxel_Jitter(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax);
    bool RayMarching_Algorithm(double3 position, double *transmittance, double3 *scatter, double step, Ray ray, double t);

    double triLinearInterpolation(double3 position, int3 voxelPosition);
    bool lightMediumIntersection(Ray ray, double t_min, double* t_max);

    double HenyeyGreenstein(double cos_theta) {
        double denominator = 1 + this->heneyGreensteinFactor * this->heneyGreensteinFactor - 2 * this->heneyGreensteinFactor * cos_theta;
        return 1 / (4 * PI) * (1 - this->heneyGreensteinFactor * this->heneyGreensteinFactor) / (denominator * std::sqrt(denominator));
    }

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

    double sigma_a = 0.03; // absorption coefficient
    double sigma_s = 0.1; // scattering coefficient

    double3 scatter = double3(0.5, 0.5, 0.5);

    Scene* scene;

    double3 background_color = double3(1, 0, 0);

    double heneyGreensteinFactor = 0;

    double d = 2;
};



