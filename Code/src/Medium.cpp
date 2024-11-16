//
// Created by Samuel on 2024-09-21.
//

#include "Medium.h"
#include "scene.h"
#include <fstream>  // Include this for file handling

// ############################## Utility Functions ##############################
double floor_epsilon(double x) {
    return std::floor(x + EPSILON);
}

double ceil_epsilon(double x) {
    return std::ceil(x - EPSILON);
}

int3 worldToVoxelCoord(double3 start, int3 voxelCounts) {
    int x = start.x < 1.0 ? start.x * voxelCounts.x : voxelCounts.x - 1;
    int y = start.y < 1.0 ? start.y * voxelCounts.y : voxelCounts.y - 1;
    int z = start.z < 1.0 ? start.z * voxelCounts.z : voxelCounts.z - 1;
    return int3(x, y, z);
}
// ###############################################################################

// ############################## Medium Class ##############################
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

    switch (traversalType) {
        case DDA:
            DDA_Traversal(start, end, hit, ray, tMin, tMax);
            break;
        case RegularStepRM:
            RayMarching_Traversal_Regular_Step(start, end, hit, ray, tMin, tMax);
            break;
        case RegularStepJitterRM:
            RayMarching_Traversal_Regular_Step_Jitter(start, end, hit, ray, tMin, tMax);
            break;
        case MiddleRayVoxelRM:
            RayMarching_Traversal_Middle_Ray_Voxel(start, end, hit, ray, tMin, tMax);
            break;
        case MiddleRayVoxelJitterRM:
            RayMarching_Traversal_Middle_Ray_Voxel_Jitter(start, end, hit, ray, tMin, tMax);
            break;
    }

    return true;
}

/**
 * Does a DDA traversal of the medium.
 *
 * This implementation is based on the paper "A Fast Voxel Traversal Algorithm for Ray Tracing"
 * by John Amanatides and Andrew Woo.
 *
 * @param start
 * @param end
 * @param hit
 * @param ray
 * @param tMin
 * @param tMax
 * @return void
 */
void Medium::DDA_Traversal(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {
    int3 entryVoxel, exitVoxel;

    entryVoxel = worldToVoxelCoord(start, this->voxelCounts);
    exitVoxel = worldToVoxelCoord(end, this->voxelCounts);

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
    // double3 accumulatedColor = transferFunction_color(this->voxels[voxelCoordinate].density);
    double accumulatedOpacity = 1;
    // accumulatedOpacity *= std::exp(0 * this->voxels[voxelCoordinate].density);

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

        auto newTmin = std::min(std::min(tMaxes.x, tMaxes.y), tMaxes.z);

        auto stepLength = newTmin - tMin;



        int coordinate = entryVoxel.x + entryVoxel.y * voxelCounts.x + entryVoxel.z * voxelCounts.x * voxelCounts.y;

        double density = this->voxels[coordinate].density;

        accumulatedOpacity *= std::exp(-stepLength * density);

        if(accumulatedOpacity >= 1) {
            // hit->accumulatedOpacity = 1.0;
            return;
        }
    }

    // hit->accumulatedOpacity = accumulatedOpacity;
}

/**
 * Does a ray marching traversal of the medium.
 *
 * This implementation traverses the medium by taking regularly spaced steps along the ray.
 *
 * @param start
 * @param end
 * @param hit
 * @param ray
 * @param tMin
 * @param tMax
 * @return void
 */
void Medium::RayMarching_Traversal_Regular_Step(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {
    int intervals = static_cast<int>(std::ceil((tMax - tMin) / stepSize));
    double step = (tMax - tMin) / intervals;

    double transmittance = 1.0;
    double3 colorResult = double3(0,0,0);

    for(int n = 0; n < intervals; n++) {
        double t = tMin + n * step;
        double3 position = ray.origin + ray.direction * t;

        if(RayMarching_Algorithm(position, &transmittance, &colorResult, step, ray, t)) {
            break;
        }
    }
    hit->transmittance = transmittance;
    hit->scatter = colorResult;
}

/**
 * Does a ray marching traversal of the medium.
 *
 * This implementation traverses the medium by taking regularly spaced steps along the ray.
 * It also adds a jitter to the step size to avoid aliasing artifacts.
 *
 * @param start
 * @param end
 * @param hit
 * @param ray
 * @param tMin
 * @param tMax
 * @return void
 */
void Medium::RayMarching_Traversal_Regular_Step_Jitter(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {
    int intervals = static_cast<int>(std::ceil((tMax - tMin) / stepSize));
    double step = (tMax - tMin) / intervals;

    double transmittance = 1.0;
    double3 colorResult = double3(0,0,0);

    for(int n = 0; n < intervals; n++) {
        double jitter = rand_double();
        double t = tMin + (n + jitter) * step;
        double3 position = ray.origin + ray.direction * t;

        if(RayMarching_Algorithm(position, &transmittance, &colorResult, step, ray, t)) {
            break;
        }
    }
    hit->transmittance = transmittance;
    hit->scatter = colorResult;
}

/**
 * Does a ray marching traversal of the medium.
 *
 * This implementation traverses the medium by taking steps from the middle of the voxel.
 *
 * @param start
 * @param end
 * @param hit
 * @param ray
 * @param tMin
 * @param tMax
 * @return void
 */
void Medium::RayMarching_Traversal_Middle_Ray_Voxel(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {
    int intervals = static_cast<int>(std::ceil((tMax - tMin) / stepSize));
    double step = (tMax - tMin) / intervals;

    double transmittance = 1.0;
    double3 colorResult = double3(0,0,0);

    for(int n = 0; n < intervals; n++) {
        double t = tMin + (n + 0.5) * step;
        double3 position = ray.origin + ray.direction * t;

        if(RayMarching_Algorithm(position, &transmittance, &colorResult, step, ray, t)) {
            break;
        }
    }
    hit->transmittance = transmittance;
    hit->scatter = colorResult;
}

/**
 * Does a ray marching traversal of the medium.
 *
 * This implementation traverses the medium by taking steps from the middle of the voxel.
 * It also adds a jitter to the step size to avoid aliasing artifacts.
 *
 * @param start
 * @param end
 * @param hit
 * @param ray
 * @param tMin
 * @param tMax
 * @return void
 */
void Medium::RayMarching_Traversal_Middle_Ray_Voxel_Jitter(double3 start, double3 end, Intersection *hit, Ray ray, double tMin, double tMax) {
    int intervals = static_cast<int>(std::ceil((tMax - tMin) / stepSize));
    double step = (tMax - tMin) / intervals;

    double transmittance = 1.0;
    double3 colorResult = double3(0,0,0);

    for(int n = 0; n < intervals; n++) {
        double jitter = rand_double() - 0.5;
        double t = tMin + (n + jitter) * step;
        double3 position = ray.origin + ray.direction * t;

        if(RayMarching_Algorithm(position, &transmittance, &colorResult, step, ray, t)) {
            break;
        }
    }
    hit->transmittance = transmittance;
    hit->scatter = colorResult;
}


/**
 * Since the algorithm for ray marching doesn't change from version to version, this function is used to
 * execute the logic of the ray marching algorithm. It will be called by the 4 different versions of ray marchers
 * (regular steps, regular steps with jitter, middle of voxel and middle of voxel with jitter).
 *
 * The reason why the function returns a boolean, is beacuse the calling ray marching function needs to know when it
 * can break from the loop. This is done when the transmittance is less than 1e-3 and the random number is less than 1/d.
 *
 * @param position
 * @param transmittance
 * @param colorResult
 * @param step
 * @param ray
 * @param t
 * @return bool
 */
bool Medium::RayMarching_Algorithm(double3 position, double *transmittance, double3 *colorResult, double step , Ray ray, double t) {
    // Generate on the fly the density of the medium using Perlin noise
    int3 voxelPosition = worldToVoxelCoord(position, this->voxelCounts);
    double density = triLinearInterpolation(position, voxelPosition);
    //double density = voxels[voxelPosition.x + voxelPosition.y * voxelCounts.x + voxelPosition.z * voxelCounts.x * voxelCounts.y].density;
    // Find the attenuation of the sample
    double sampleAttenuation = std::exp(-step * this->sigma_t * density);

    *transmittance *= sampleAttenuation;

    for(auto light: this->scene->lights) {
        double3 lightPosition = mul(i_transform, {light.position, 1}).xyz();
        Ray lightRay = Ray(position, normalize(lightPosition - position));
        double t_light;

        if(lightMediumIntersection(lightRay, t, &t_light)) {
            double cos_theta = dot(ray.direction, lightRay.direction);
            double phaseFunction = HenyeyGreenstein(cos_theta);
            double lightAttenuation = std::exp(-t_light * this->sigma_t * density);
            *colorResult += *transmittance * light.emission * lightAttenuation * step * this->sigma_s * density * phaseFunction;
        }

        if(*transmittance < 1e-3) {
            if(rand() < 1.f/this->d) {
                return true;
            }
            else {
                *transmittance *= this->d;
            }
        }
    }

    return false;
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

bool Medium::lightMediumIntersection(Ray ray, double t_min, double* t_max) {
    double3 inv_dir = double3(1/ray.direction.x, 1/ray.direction.y, 1/ray.direction.z);

    double tmin, tmax, tymin, tymax, tzmin, tzmax;

    tmin = (minBound.x - ray.origin.x) * inv_dir.x;
    tmax = (maxBound.x - ray.origin.x) * inv_dir.x;
    tymin = (minBound.y - ray.origin.y) * inv_dir.y;
    tymax = (maxBound.y - ray.origin.y) * inv_dir.y;

    if ((tmin > tymax) || (tymin > tmax)) return false;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    tzmin = (minBound.z - ray.origin.z) * inv_dir.z;
    tzmax = (maxBound.z - ray.origin.z) * inv_dir.z;

    if ((tmin > tzmax) || (tzmin > tmax)) return false;
    if (tzmin > tmin) tmin = tzmin;
    if (tzmax < tmax) tmax = tzmax;

    *t_max = tmax;

    return true;
}

