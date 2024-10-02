//
// Created by Samuel on 2024-09-21.
//

#include "Medium.h"
#include <fstream>  // Include this for file handling

bool Medium::local_intersect(Ray ray, double t_min, double t_max, Intersection *hit) {

    /*
     * Compute intersection with cube.
     * Bottom back left corner is at (0,0,0)
     * Top front right corner is at (1,1,1)
     */

    // Inverse ray direction to avoid division by zero
    double3 inv_dir = {1.0 / ray.direction.x, 1.0 / ray.direction.y, 1.0 / ray.direction.z};

    double3 t0 = (double3(0,0,0) - ray.origin) * inv_dir;
    double3 t1 = (double3(1,1,1) - ray.origin) * inv_dir;

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

    std::ofstream outfile("intersection_points.txt", std::ios::app);  // Open in append mode

    if (outfile.is_open()) {
        outfile << "Start point: (" << start.x << ", " << start.y << ", " << start.z << ")\n";
        outfile << "End point: (" << end.x << ", " << end.y << ", " << end.z << ")\n";
        outfile << "---------------------------\n";  // Separator for clarity
        outfile.close();
    } else {
        std::cerr << "Error opening file for writing\n";
    }

    hit->position = start;
    hit->depth = tmin_max;

    return DDA(start, end, hit);
}

bool Medium::DDA(double3 start, double3 end, Intersection *hit) {
    float sizeOfVoxelX = 1.0 / voxel_x;
    float sizeOfVoxelY = 1.0 / voxel_y;
    float sizeOfVoxelZ = 1.0 / voxel_z;

    // Calculate initial voxel indices
    int startVoxelX = std::clamp(static_cast<int>(std::floor(start.x / sizeOfVoxelX)), 0, voxel_x - 1);
    int startVoxelY = std::clamp(static_cast<int>(std::floor(start.y / sizeOfVoxelY)), 0, voxel_y - 1);
    int startVoxelZ = std::clamp(static_cast<int>(std::floor(start.z / sizeOfVoxelZ)), 0, voxel_z - 1);

    int endVoxelX = std::clamp(static_cast<int>(std::floor(end.x / sizeOfVoxelX)), 0, voxel_x - 1);
    int endVoxelY = std::clamp(static_cast<int>(std::floor(end.y / sizeOfVoxelY)), 0, voxel_y - 1);
    int endVoxelZ = std::clamp(static_cast<int>(std::floor(end.z / sizeOfVoxelZ)), 0, voxel_z - 1);

    // Determine step direction and calculate tMax and tDelta for each axis
    int stepX = (end.x > start.x) ? 1 : (end.x < start.x) ? -1 : 0;
    int stepY = (end.y > start.y) ? 1 : (end.y < start.y) ? -1 : 0;
    int stepZ = (end.z > start.z) ? 1 : (end.z < start.z) ? -1 : 0;

    double tMaxX = (stepX != 0) ? ((stepX > 0 ? (startVoxelX + 1) * sizeOfVoxelX : startVoxelX * sizeOfVoxelX) - start.x) / (end.x - start.x) : std::numeric_limits<double>::max();
    double tMaxY = (stepY != 0) ? ((stepY > 0 ? (startVoxelY + 1) * sizeOfVoxelY : startVoxelY * sizeOfVoxelY) - start.y) / (end.y - start.y) : std::numeric_limits<double>::max();
    double tMaxZ = (stepZ != 0) ? ((stepZ > 0 ? (startVoxelZ + 1) * sizeOfVoxelZ : startVoxelZ * sizeOfVoxelZ) - start.z) / (end.z - start.z) : std::numeric_limits<double>::max();

    double tDeltaX = (stepX != 0) ? std::abs(sizeOfVoxelX / (end.x - start.x)) : std::numeric_limits<double>::max();
    double tDeltaY = (stepY != 0) ? std::abs(sizeOfVoxelY / (end.y - start.y)) : std::numeric_limits<double>::max();
    double tDeltaZ = (stepZ != 0) ? std::abs(sizeOfVoxelZ / (end.z - start.z)) : std::numeric_limits<double>::max();

    // Initialize voxel indices
    int x = startVoxelX, y = startVoxelY, z = startVoxelZ;

    do {
        // Debug output
        std::ofstream outfile("voxel_traversal_debug.txt", std::ios::app);
        if (outfile.is_open()) {
            outfile << "Current voxel: (" << x << ", " << y << ", " << z << ")\n";
            outfile << "tMaxX: " << tMaxX << ", tMaxY: " << tMaxY << ", tMaxZ: " << tMaxZ << "\n";
            outfile << "stepX: " << stepX << ", stepY: " << stepY << ", stepZ: " << stepZ << "\n";
        }

        // Check if the current voxel is non-empty
        Voxel currentVoxel = voxels[x + y * voxel_x + z * voxel_x * voxel_y];
        if (currentVoxel.density > 0.0) {
            // Set the intersection depth and return true
            hit->depth = std::min(tMaxX, std::min(tMaxY, tMaxZ));
            return true;
        }

        // Update voxel based on which axis has the smallest tMax
        if (tMaxX < tMaxY && tMaxX < tMaxZ) {
            x += stepX;
            tMaxX += tDeltaX;
        } else if (tMaxY < tMaxZ) {
            y += stepY;
            tMaxY += tDeltaY;
        } else {
            z += stepZ;
            tMaxZ += tDeltaZ;
        }

        // Close the debug output
        if (outfile.is_open()) outfile.close();

        // Check if we are out of bounds
        if (x < 0 || x >= voxel_x || y < 0 || y >= voxel_y || z < 0 || z >= voxel_z) {
            return false;
        }

    } while (x != endVoxelX || y != endVoxelY || z != endVoxelZ);

    // If we exit the loop without finding a voxel, return false
    return false;
}

bool Medium::drawLine(int3 start, int3 end, Intersection *hit) {
    int x0 = start.x;
    int y0 = start.y;
    int z0 = start.z;

    int x1 = end.x;
    int y1 = end.y;
    int z1 = end.z;

    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int dz = std::abs(z1 - z0);

    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int sz = z0 < z1 ? 1 : -1;

    double hypothenuse = sqrt((dx * dx) + (dy * dy) + (dz * dz));

    double tMaxX = hypothenuse * 0.5 / dx;
    double tMaxY = hypothenuse * 0.5 / dy;
    double tMaxZ = hypothenuse * 0.5 / dz;

    double tDelatX = hypothenuse / dx;
    double tDelatY = hypothenuse / dy;
    double tDelatZ = hypothenuse / dz;

    Voxel currentVoxel = voxels[x0 + y0 * voxel_x + z0 * voxel_x * voxel_y];

    while(x0 != x1 || y0 != y1 || z0 != z1) {
        if(tMaxX < tMaxY) {
            if(tMaxX < tMaxZ) {
                x0 = x0 + sx;
                tMaxX = tMaxX + tDelatX;
            }
            else if(tMaxX > tMaxZ) {
                z0 = z0 + sz;
                tMaxZ = tMaxZ + tDelatZ;
            }
            else {
                x0 = x0 + sx;
                z0 = z0 + sz;
                tMaxX = tMaxX + tDelatX;
                tMaxZ = tMaxZ + tDelatZ;
            }
        }
        else if(tMaxX > tMaxY) {
            if(tMaxY < tMaxZ) {
                y0 = y0 + sy;
                tMaxY = tMaxY + tDelatY;
            }
            else if(tMaxY > tMaxZ) {
                z0 = z0 + sz;
                tMaxZ = tMaxZ + tDelatZ;
            }
            else {
                y0 = y0 + sy;
                z0 = z0 + sz;
                tMaxY = tMaxY + tDelatY;
                tMaxZ = tMaxZ + tDelatZ;
            }
        }
        else {
            if(tMaxX < tMaxZ) {
                x0 = x0 + sx;
                y0 = y0 + sy;
                tMaxX = tMaxX + tDelatX;
                tMaxY = tMaxY + tDelatY;
            }
            else if(tMaxX > tMaxZ) {
                z0 = z0 + sz;
                tMaxZ = tMaxZ + tDelatZ;
            }
            else {
                x0 = x0 + sx;
                y0 = y0 + sy;
                z0 = z0 + sz;
                tMaxX = tMaxX + tDelatX;
                tMaxY = tMaxY + tDelatY;
                tMaxZ = tMaxZ + tDelatZ;
            }
        }

        if(currentVoxel.density > 0.0) {
            float firstDepth = std::min(tMaxX, std::min(tMaxY, tMaxZ));
            hit->depth = firstDepth;
            return true;
        }

        currentVoxel = voxels[x0 + y0 * voxel_x + z0 * voxel_x * voxel_y];
    }
    return false;
}
