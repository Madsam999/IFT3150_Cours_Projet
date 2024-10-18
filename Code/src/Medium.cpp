//
// Created by Samuel on 2024-09-21.
//

#include "Medium.h"
#include <fstream>  // Include this for file handling

// ############################## Utility Functions ##############################
double floor_epsilon(double x) {
    return std::floor(x + EPSILON);
}

double ceil_epsilon(double x) {
    return std::ceil(x - EPSILON);
}

int3 calculateEntryVoxel(double3 start, int3 voxelCounts) {
    int x = start.x < 1.0 ? start.x * voxelCounts.x : voxelCounts.x - 1;
    int y = start.y < 1.0 ? start.y * voxelCounts.y : voxelCounts.y - 1;
    int z = start.z < 1.0 ? start.z * voxelCounts.z : voxelCounts.z - 1;
    return int3(x, y, z);
}

int3 calculateExitVoxel(double3 end, int3 voxelCounts) {
    int x = end.x < 1.0 ? end.x * voxelCounts.x : voxelCounts.x - 1;
    int y = end.y < 1.0 ? end.y * voxelCounts.y : voxelCounts.y - 1;
    int z = end.z < 1.0 ? end.z * voxelCounts.z : voxelCounts.z - 1;
    return int3(x, y, z);
}
// ###############################################################################

// ############################## Voxel Class ##############################
Voxel::Voxel(double density): density(density) {
    this->density = density;
}
// #########################################################################

// ############################## Medium Class ##############################
void Medium::setup_transform(double4x4 m) {
    this->transform = m;
    this->i_transform = inverse(m);
}

Medium::Medium(int voxel_x, int voxel_y, int voxel_z, int traversalType) {
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
    this->stepSize = 1/1000.0;
}

bool Medium::intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
    Ray lray{mul(i_transform, {ray.origin,1}).xyz(), mul(i_transform, {ray.direction,0}).xyz()};
    return local_intersect(lray, t_min, t_max, hit);
}

void Medium::createVoxels() {
    for(int x = 0; x < voxelCounts.x; x++) {
        for (int y = 0; y < voxelCounts.y; y++) {
            for (int z = 0; z < voxelCounts.z; z++) {
                double density = rand_double();
                this->voxels[x + y * voxelCounts.x + z * voxelCounts.x * voxelCounts.y] = Voxel(density);
            }
        }
    }
}

double3 Medium::transferFunction_color(double density) const {
    return mediumColor;
}

double Medium::transferFunction_opacity(double density) const {
    return density;
}

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


    auto tMin = tmin_max;
    auto tMax = tmax_min;

    switch (this->traversalType) {
        case 0:
            DDA_Traversal(start, end, hit, ray, tMin, tMax);
            break;
        case 1:
            RayMarching_Traversal(start, end, hit, ray, tMin, tMax);
            break;
        case 2:
            RayMarching_Traversal(start, end, hit, ray, tMin, tMax);
            break;
        case 3:
            RayMarching_Traversal(start, end, hit, ray, tMin, tMax);
            break;
        case 4:
            RayMarching_Traversal(start, end, hit, ray, tMin, tMax);
            break;

    }

    hit->length = tmax_min - tmin_max;
    hit->scatter = this->mediumColor;

    return true;
}

void Medium::DDA_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {
    int3 entryVoxel, exitVoxel;

    entryVoxel = calculateEntryVoxel(start, this->voxelCounts);
    exitVoxel = calculateExitVoxel(end, this->voxelCounts);

    double3 tDeltas;
    tDeltas.x = ray.direction.x != 0.0 ? this->voxelSize.x / std::abs(ray.direction.x) : DBL_MAX;
    tDeltas.y = ray.direction.y != 0.0 ? this->voxelSize.y / std::abs(ray.direction.y) : DBL_MAX;
    tDeltas.z = ray.direction.z != 0.0 ? this->voxelSize.z / std::abs(ray.direction.z) : DBL_MAX;

    double3 tMaxes;
    tMaxes.x = ray.direction.x > 0.0 ? ((entryVoxel.x + 1) * this->voxelSize.x - ray.origin.x) / ray.direction.x :
                                       ray.direction.x < 0.0 ?
                                       ((entryVoxel.x) * this->voxelSize.x - ray.origin.x) / ray.direction.x : DBL_MAX;
    tMaxes.y = ray.direction.y > 0.0 ? ((entryVoxel.y + 1) * this->voxelSize.y - ray.origin.y) / ray.direction.y :
                                       ray.direction.y < 0.0 ?
                                       ((entryVoxel.y) * this->voxelSize.y - ray.origin.y) / ray.direction.y : DBL_MAX;
    tMaxes.z = ray.direction.z > 0.0 ? ((entryVoxel.z + 1) * this->voxelSize.z - ray.origin.z) / ray.direction.z :
                                       ray.direction.z < 0.0 ?
                                       ((entryVoxel.z) * this->voxelSize.z - ray.origin.z) / ray.direction.z : DBL_MAX;

    int voxelCoordinate = entryVoxel.x + entryVoxel.y * voxelCounts.x + entryVoxel.z * voxelCounts.x * voxelCounts.y;
    double3 accumulatedColor = transferFunction_color(this->voxels[voxelCoordinate].density);
    double accumulatedOpacity = transferFunction_opacity(this->voxels[voxelCoordinate].density);

    while(entryVoxel.x != exitVoxel.x || entryVoxel.y != exitVoxel.y || entryVoxel.z != exitVoxel.z) {
        if(tMaxes.x < tMaxes.y && tMaxes.x < tMaxes.z) {
            entryVoxel.x += ray.direction.x > 0 ? 1 : -1;
            tMaxes.x += tDeltas.x;
        }
        else if(tMaxes.y < tMaxes.z) {
            entryVoxel.y += ray.direction.y > 0 ? 1 : -1;
            tMaxes.y += tDeltas.y;
        }
        else {
            entryVoxel.z += ray.direction.z > 0 ? 1 : -1;
            tMaxes.z += tDeltas.z;
        }

        int coordinate = entryVoxel.x + entryVoxel.y * voxelCounts.x + entryVoxel.z * voxelCounts.x * voxelCounts.y;

        double density = this->voxels[coordinate].density;

        accumulatedOpacity += transferFunction_opacity(density) * (1 - accumulatedOpacity);

        if(accumulatedOpacity >= 1) {
            hit->accumulatedOpacity = 1.0;
            return;
        }
    }

    hit->accumulatedOpacity = accumulatedOpacity;
}

void Medium::RayMarching_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {

    int intervals = static_cast<int>(std::ceil((tMax - tMin) / this->stepSize));

    auto t0 = tMin;
    auto t1 = tMax;

    auto stepSize = (t1 - t0) / intervals;

    float transparancy = 1.0;

    for(int i = 0; i < intervals; i++) {

        auto t = t0 + (stepSize * i);

        double3 position = ray.origin + ray.direction * t;

        int3 voxelPosition = calculateEntryVoxel(position, this->voxelCounts);
        int voxelCoordinate = voxelPosition.x + voxelPosition.y * voxelCounts.x + voxelPosition.z * voxelCounts.x * voxelCounts.y;

        double density = triLinearInterpolation(position, voxelPosition);

        double attenuation = std::exp(-stepSize * transferFunction_opacity(density));
        transparancy *= attenuation;

        if(transparancy <= EPSILON) {
            hit->accumulatedOpacity = 1.0;
            return;
        }
    }

    hit->accumulatedOpacity = 1.0 - transparancy;

    return;
}

double Medium::triLinearInterpolation(double3 position, int3 voxelPosition) {
    int x[2], y[2], z[2];

    x[0] = voxelPosition.x;
    y[0] = voxelPosition.y;
    z[0] = voxelPosition.z;

    if(voxelPosition.x + 1 >= voxelCounts.x) {
        x[1] = voxelPosition.x - 1;
    }
    else if(voxelPosition.x - 1 < 0) {
        x[1] = voxelPosition.x + 1;
    }
    else {
        double leftVoxelPos = (voxelPosition.x - 1) * voxelSize.x;
        double rightVoxelPos = (voxelPosition.x + 1) * voxelSize.x;

        if(std::abs(leftVoxelPos - position.x) < std::abs(rightVoxelPos - position.x)) {
            x[1] = voxelPosition.x - 1;
        }
        else {
            x[1] = voxelPosition.x + 1;
        }
    }
    if(voxelPosition.y + 1 >= voxelCounts.y) {
        y[1] = voxelPosition.y - 1;
    }
    else if(voxelPosition.y - 1 < 0) {
        y[1] = voxelPosition.y + 1;
    }
    else {
        double bottomVoxelPos = (voxelPosition.y - 1) * voxelSize.y;
        double topVoxelPos = (voxelPosition.y + 1) * voxelSize.y;

        if(std::abs(bottomVoxelPos - position.y) < std::abs(topVoxelPos - position.y)) {
            y[1] = voxelPosition.y - 1;
        }
        else {
            y[1] = voxelPosition.y + 1;
        }
    }
    if(voxelPosition.z + 1 >= voxelCounts.z) {
        z[1] = voxelPosition.z - 1;
    }
    else if(voxelPosition.z - 1 < 0) {
        z[1] = voxelPosition.z + 1;
    }
    else {
        double backVoxelPos = (voxelPosition.z - 1) * voxelSize.z;
        double frontVoxelPos = (voxelPosition.z + 1) * voxelSize.z;

        if(std::abs(backVoxelPos - position.z) < std::abs(frontVoxelPos - position.z)) {
            z[1] = voxelPosition.z - 1;
        }
        else {
            z[1] = voxelPosition.z + 1;
        }
    }

    int3 M1, M2, M3, M4, M5, M6, M7, M8;

    M1 = int3(x[0], y[0], z[0]);
    M2 = int3(x[1], y[0], z[0]);

    M3 = int3(x[0], y[1], z[0]);
    M4 = int3(x[1], y[1], z[0]);

    M5 = int3(x[0], y[0], z[1]);
    M6 = int3(x[1], y[0], z[1]);

    M7 = int3(x[0], y[1], z[1]);
    M8 = int3(x[1], y[1], z[1]);

    double3 P1, P2, P3, P4, P5, P6, P7, P8;

    P1 = double3(M1.x * voxelSize.x, M1.y * voxelSize.y, M1.z * voxelSize.z);
    P2 = double3(M2.x * voxelSize.x, M2.y * voxelSize.y, M2.z * voxelSize.z);

    P3 = double3(M3.x * voxelSize.x, M3.y * voxelSize.y, M3.z * voxelSize.z);
    P4 = double3(M4.x * voxelSize.x, M4.y * voxelSize.y, M4.z * voxelSize.z);

    P5 = double3(M5.x * voxelSize.x, M5.y * voxelSize.y, M5.z * voxelSize.z);
    P6 = double3(M6.x * voxelSize.x, M6.y * voxelSize.y, M6.z * voxelSize.z);

    P7 = double3(M7.x * voxelSize.x, M7.y * voxelSize.y, M7.z * voxelSize.z);
    P8 = double3(M8.x * voxelSize.x, M8.y * voxelSize.y, M8.z * voxelSize.z);

    double xf = position.x;
    double x0 = P1.x;
    double x1 = P2.x;

    double tx = (xf - x0) / (x1 - x0);

    double3 P12, P34, P56, P78;

    P12 = P1 * (1 - tx) + P2 * tx;
    P34 = P3 * (1 - tx) + P4 * tx;
    P56 = P5 * (1 - tx) + P6 * tx;
    P78 = P7 * (1 - tx) + P8 * tx;

    double zf = position.z;
    double z0 = P12.z;
    double z1 = P56.z;

    double tz = (zf - z0) / (z1 - z0);

    double3 P9, P10;

    P9 = P12 * (1 - tz) + P56 * tz;
    P10 = P34 * (1 - tz) + P78 * tz;

    double yf = position.y;
    double y0 = P9.y;
    double y1 = P10.y;

    double ty = (yf - y0) / (y1 - y0);

    double D1, D2, D3, D4, D5, D6, D7, D8;
    D1 = voxels[M1.x + M1.y * voxelCounts.x + M1.z * voxelCounts.x * voxelCounts.y].density;
    D2 = voxels[M2.x + M2.y * voxelCounts.x + M2.z * voxelCounts.x * voxelCounts.y].density;
    D3 = voxels[M3.x + M3.y * voxelCounts.x + M3.z * voxelCounts.x * voxelCounts.y].density;
    D4 = voxels[M4.x + M4.y * voxelCounts.x + M4.z * voxelCounts.x * voxelCounts.y].density;
    D5 = voxels[M5.x + M5.y * voxelCounts.x + M5.z * voxelCounts.x * voxelCounts.y].density;
    D6 = voxels[M6.x + M6.y * voxelCounts.x + M6.z * voxelCounts.x * voxelCounts.y].density;
    D7 = voxels[M7.x + M7.y * voxelCounts.x + M7.z * voxelCounts.x * voxelCounts.y].density;
    D8 = voxels[M8.x + M8.y * voxelCounts.x + M8.z * voxelCounts.x * voxelCounts.y].density;

    double D12, D34, D56, D78;

    D12 = D1 * (1 - tx) + D2 * tx;
    D34 = D3 * (1 - tx) + D4 * tx;
    D56 = D5 * (1 - tx) + D6 * tx;
    D78 = D7 * (1 - tx) + D8 * tx;

    double D9, D10;

    D9 = D12 * (1 - tz) + D56 * tz;
    D10 = D34 * (1 - tz) + D78 * tz;

    double Df;

    Df = D9 * (1 - ty) + D10 * ty;
    return Df;
}

double Medium::testLightIntersection(Ray ray, double t_min, double t_max, Intersection *hit) {
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


    auto tMin = tmin_max;
    auto tMax = tmax_min;
}

