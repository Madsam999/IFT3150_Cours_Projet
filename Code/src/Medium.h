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
    Voxel(float density) {
        this->density = density;
    }

    float density;
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
    Medium(int voxel_x, int voxel_y, int voxel_z) {
        this->voxel_x = voxel_x;
        this->voxel_y = voxel_y;
        this->voxel_z = voxel_z;
        this->voxels = std::vector<Voxel>(voxel_x * voxel_y * voxel_z);
        createVoxels();
    };

    int voxel_x;
    int voxel_y;
    int voxel_z;

    std::vector<double3> bounds;

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

    void printMatrix(double4x4 m)
    {
        std::cout << m.x.x << m.y.x << m.z.x << m.w.x << std::endl;
        std::cout << m.x.y << m.y.y << m.z.y << m.w.y << std::endl;
        std::cout << m.x.z << m.y.z << m.z.z << m.w.z << std::endl;
        std::cout << m.x.w << m.y.w << m.z.w << m.w.w << std::endl;
    }

    void printVector(double3 v)
    {
        std::cout << "x:" << v.x << "y:" << v.y << "z:" << v.z << std::endl;
    }

    void createVoxels() {
        for(int x = 0; x < voxel_x; x++) {
            for (int y = 0; y < voxel_y; y++) {
                for (int z = 0; z < voxel_z; z++) {
                    voxels[x + y * voxel_x + z * voxel_x * voxel_y] = Voxel(0);
                }
            }
        }
    }

    bool DDA(double3 start, double3 end, Intersection *hit);

};

