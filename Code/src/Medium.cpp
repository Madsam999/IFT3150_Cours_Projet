//
// Created by Samuel on 2024-09-21.
//

#include "Medium.h"
#include <fstream>  // Include this for file handling

double floor_epsilon(double x) {
    return std::floor(x + EPSILON);
}

double ceil_epsilon(double x) {
    return std::ceil(x - EPSILON);
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

    if(traversalType == RayMarching) {
        RayMarching_Traversal(start, end, hit, ray, tMin, tMax);
    }
    else {
        DDA_Traversal(start, end, hit, ray, tMin, tMax);
    }

    return true;
}

void Medium::DDA_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {

    // double3 test = ray.origin + ray.direction * 50;

    // std::cout << "Test: " << test.x << " " << test.y << " " << test.z << std::endl;

    int3 entryVoxels, exitVoxels;
    if(start.x < 1.0) {
        entryVoxels.x = start.x * voxelCounts.x;
    }
    else {
        entryVoxels.x = voxelCounts.x - 1;
    }
    if(start.y < 1.0) {
        entryVoxels.y = start.y * voxelCounts.y;
    }
    else {
        entryVoxels.y = voxelCounts.y - 1;
    }
    if(start.z < 1.0) {
        entryVoxels.z = start.z * voxelCounts.z;
    }
    else {
        entryVoxels.z = voxelCounts.z - 1;
    }

    if(end.x < 1.0) {
        exitVoxels.x = end.x * voxelCounts.x;
    }
    else {
        exitVoxels.x = voxelCounts.x - 1;
    }
    if(end.y < 1.0) {
        exitVoxels.y = end.y * voxelCounts.y;
    }
    else {
        exitVoxels.y = voxelCounts.y - 1;
    }
    if(end.z < 1.0) {
        exitVoxels.z = end.z * voxelCounts.z;
    }
    else {
        exitVoxels.z = voxelCounts.z - 1;
    }

    if((entryVoxels == int3(0,0,1)) && (exitVoxels == int3(0, 0,0))) {
        // std::cout << "EntryVoxels: " << entryVoxels.x << " " << entryVoxels.y << " " << entryVoxels.z << std::endl;
    }

    // if(voxels[entryVoxels.x + entryVoxels.y * voxelCounts.x + entryVoxels.z * voxelCounts.x * voxelCounts.y].density > 0) {
    //    // hit->position = {entryVoxels.x * voxelSize.x, entryVoxels.y * voxelSize.y, entryVoxels.z * voxelSize.z};
    //    return true;
    //}

    double entryDensity = voxels[entryVoxels.x + entryVoxels.y * voxelCounts.x + entryVoxels.z * voxelCounts.x * voxelCounts.y].density;

    // Colour accumulated along the ray inside the volume grid
    double3 accumulatedColor = transferFunction_color(entryDensity);

    // Opacity accumulated along the ray inside the volume grid
    double accumulatedOpacity = transferFunction_opacity(triLinearInterpolation(start, entryVoxels));

    while(entryVoxels.x != exitVoxels.x || entryVoxels.y != exitVoxels.y || entryVoxels.z != exitVoxels.z) {
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

        double test = std::floor(newStart.x * voxelCounts.x);

        if(ray.direction.x > 0) {
            entryVoxels.x = newStart.x < 1 ? floor_epsilon(newStart.x * voxelCounts.x) : voxelCounts.x - 1;
        }
        else if(ray.direction.x < 0) {
            entryVoxels.x = newStart.x < 1 ? ceil_epsilon(newStart.x * static_cast<double>(voxelCounts.x)) - 1 : 0;
        }
        if(ray.direction.y > 0) {
            entryVoxels.y = newStart.y < 1 ? floor_epsilon(newStart.y * voxelCounts.y) : voxelCounts.y - 1;
        }
        else if(ray.direction.y < 0) {
            entryVoxels.y = newStart.y < 1 ? ceil_epsilon(newStart.y * static_cast<double>(voxelCounts.y)) - 1 : 0;
        }
        if(ray.direction.z > 0) {
            entryVoxels.z = newStart.z < 1 ? floor_epsilon(newStart.z * voxelCounts.z) : voxelCounts.z - 1;
        }
        else if(ray.direction.z < 0) {
            entryVoxels.z = newStart.z < 1 ? ceil_epsilon(newStart.z * static_cast<double>(voxelCounts.z)) - 1 : 0;
        }

        double voxelDensity = voxels[entryVoxels.x + entryVoxels.y * voxelCounts.x + entryVoxels.z * voxelCounts.x * voxelCounts.y].density;

        accumulatedColor = accumulatedColor + (1 - accumulatedOpacity) * transferFunction_opacity(voxelDensity) * transferFunction_color(voxelDensity);
        accumulatedOpacity = accumulatedOpacity + (1 - accumulatedOpacity) * transferFunction_opacity(triLinearInterpolation(newStart, entryVoxels));

        if(accumulatedOpacity >= 1) {
            hit->accumulatedOpacity = accumulatedOpacity;
            hit->accumulatedColor = accumulatedColor;
            break;
        }

    }
    hit->accumulatedOpacity = accumulatedOpacity;
    hit->accumulatedColor = accumulatedColor;
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

