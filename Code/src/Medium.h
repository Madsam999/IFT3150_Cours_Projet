//
// Created by Samuel on 2024-09-21.
//

#include "linalg/linalg.h"
using namespace linalg::aliases;

#ifndef RAY_MEDIUM_H
#define RAY_MEDIUM_H

#endif //RAY_MEDIUM_H

class Medium {
public:
    Medium() {};
    Medium(int voxel_x, int voxel_y, int voxel_z, double3 position) {};

    int voxel_x;
    int voxel_y;
    int voxel_z;

    double3 position;


};

class Voxel {
public:
    Voxel() {};
    Voxel(float density, int3 position) {};

    float density;
    int3 position;
};