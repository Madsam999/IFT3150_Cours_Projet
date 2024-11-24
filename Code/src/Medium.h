//
// Created by Samuel on 2024-09-21.
//

#pragma once

#include <iosfwd>
#include <iomanip>
#include "linalg/linalg.h"
#include "basic.h"
#include "aabb.h"
#include "object.h"
#include "pcg32/pcg32.h"

using namespace linalg::aliases;

class Scene;

/**
 * @brief Structure représentant un voxel. Un voxel est un point dans l'espace 3D avec une densitée.
 */
struct voxel {
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
        this->voxels = std::vector<voxel>(voxelCounts.x * voxelCounts.y * voxelCounts.z);
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
        for (size_t i = 0; i < 256; i++)
            p[256 + i] = p[i] = permutation[i];

        this->stepSize = 0.05;
        // Read density values from text file
        makeShereInGrid(voxel_x);
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
                    voxel v;
                    v.density = eval_density(double3((x + 0.5) / voxelCounts.x, (y + 0.5) / voxelCounts.y, (z + 0.5) / voxelCounts.z));
                    this->voxels[x + y * voxelCounts.x + z * voxelCounts.x * voxelCounts.y] = v;
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

    double fade(double t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    double lerp(double t, double a, double b) {
        return a + t * (b - a);
    }

    double grad(int hash, double x, double y, double z) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }

    double noise(double3 position) {
        double x = position.x,
               y = position.y,
               z = position.z;

        int X = (int)floor(x) & 255,
            Y = (int)floor(y) & 255,
            Z = (int)floor(z) & 255;
        x -= floor(x);
        y -= floor(y);
        z -= floor(z);
        double u = fade(x), v = fade(y), w = fade(z);
        int A = p[X] + Y, AA = p[A] + Z, AB = p[A + 1] + Z, B = p[X + 1] + Y, BA = p[B] + Z, BB = p[B + 1] + Z;
        return lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z)),
                            lerp(u, grad(p[AB], x, y - 1, z), grad(p[BB], x - 1, y - 1, z))),
                    lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1), grad(p[BA + 1], x - 1, y, z - 1)),
                         lerp(u, grad(p[AB + 1], x, y - 1, z - 1), grad(p[BB + 1], x - 1, y - 1, z - 1))));
    }

    void makeShereInGrid(int radius) {
        for (int x = 0; x < voxelCounts.x; x++) {
            for (int y = 0; y < voxelCounts.y; y++) {
                for (int z = 0; z < voxelCounts.z; z++) {
                    double3 position;
                    position.x = (x + 0.5) / voxelCounts.x;
                    position.y = (y + 0.5) / voxelCounts.y;
                    position.z = (z + 0.5) / voxelCounts.z;
                    double3 center = double3(0.5, 0.5, 0.5);
                    double distance = length(position - center);
                    voxel v;
                    v.density = 0;
                    voxels[x + y * voxelCounts.x + z * voxelCounts.x * voxelCounts.y] = v;
                    if (distance < 0.5) {
                        double density = eval_density(position);
                        voxels[x + y * voxelCounts.x + z * voxelCounts.x * voxelCounts.y].density = density;
                    }
                }
            }
        }
    }

    double smoothstep(double edge0, double edge1, double x) {
        double t = std::clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        return t * t * (3 - 2 * t);
    }

    /*
     * Only works for a circle right now, since that's how Mr. Scratch wants it!
     */
    double eval_density(double3 position) {
        double3 vp = position - center;
        double3 vp_xform;

        // Rotate the point around the y-axis
        double theta = PI/4;
        vp_xform.x = vp.x * cos(theta) - vp.z * sin(theta);
        vp_xform.y = vp.y;
        vp_xform.z = vp.x * sin(theta) + vp.z * cos(theta);

        // double dist = std::min(1.0, length(vp_xform) / 0.5);
        //double falloff = smoothstep(0.8, 1, dist);
        double freq = 0.5;
        float lacunarity = 2.7;
        float H = 0.4;
        size_t octaves = 5;
        vp_xform *= freq;
        float fbmResult = 0;
        double3 offset = double3(0.1, 0.15, 0.25);
        for (size_t k = 0; k < octaves; k++) {
            fbmResult += noise(vp_xform) * pow(lacunarity, -H * k);
            vp_xform *= lacunarity;
        }
        return std::max(0.f, fbmResult);//(1 - falloff);//std::max(0.f, fbmResult);// * (1 - falloff));
    }

    void writeDensityToFile() {
        std::ofstream file ("test_grid_2.txt");

        if(!file) {
            std::cerr << "Could not open file." << std::endl;
            return;
        }

        file << std::fixed << std::setprecision(7);

        for(int x = 0; x < voxelCounts.x; x++) {
            for (int y = 0; y < voxelCounts.y; y++) {
                for (int z = 0; z < voxelCounts.z; z++) {
                    double3 position;
                    position.x = (x + 0.5) / voxelCounts.x;
                    position.y = (y + 0.5) / voxelCounts.y;
                    position.z = (z + 0.5) / voxelCounts.z;
                    file << eval_density(position) << std::endl;
                }
            }
        }

        file.close();
    }

    void readDensityFromFile() {
        std::ifstream file ("test_grid_2.txt");

        if(!file.is_open()) {
            std::cerr << "Could not open file." << std::endl;
            return;
        }

        std::string line;
        int i = 0;
        while (std::getline(file, line)) {
            voxels[i].density = std::stod(line);
            i++;
        }

        file.close();
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

    std::vector<voxel> voxels;

    double3 minBound = double3(0,0,0);
    double3 maxBound = double3(1,1,1);

    double3 voxelSize;

    double3 mediumColor;

    double4x4 transform;
    double4x4 i_transform;

    double sigma_a = 0.01; // absorption coefficient
    double sigma_s = 0.5; // scattering coefficient
    double sigma_t = sigma_a + sigma_s; // extinction coefficient

    double3 scatter = double3(0.5, 0.5, 0.5);

    Scene* scene;

    double3 background_color = double3(1, 0, 0);

    double heneyGreensteinFactor = 0;

    double d = 2;

    double falloff = 4;

    int permutation[256] = {
            151, 160, 137,  91,  90,  15, 131,  13, 201,  95,  96,  53, 194, 233,   7, 225,
            140,  36, 103,  30,  69, 142,   8,  99,  37, 240,  21,  10,  23, 190,   6, 148,
            247, 120, 234,  75,   0,  26, 197,  62,  94, 252, 219, 203, 117,  35,  11,  32,
            57, 177,  33,  88, 237, 149,  56,  87, 174,  20, 125, 136, 171, 168,  68, 175,
            74, 165,  71, 134, 139,  48,  27, 166,  77, 146, 158, 231,  83, 111, 229, 122,
            60, 211, 133, 230, 220, 105,  92,  41,  55,  46, 245,  40, 244, 102, 143,  54,
            65,  25,  63, 161,   1, 216,  80,  73, 209,  76, 132, 187, 208,  89,  18, 169,
            200, 196, 135, 130, 116, 188, 159,  86, 164, 100, 109, 198, 173, 186,   3,  64,
            52, 217, 226, 250, 124, 123,   5, 202,  38, 147, 118, 126, 255,  82,  85, 212,
            207, 206,  59, 227,  47,  16,  58,  17, 182, 189,  28,  42, 223, 183, 170, 213,
            119, 248, 152,   2,  44, 154, 163,  70, 221, 153, 101, 155, 167,  43, 172,   9,
            129,  22,  39, 253,  19,  98, 108, 110,  79, 113, 224, 232, 178, 185, 112, 104,
            218, 246,  97, 228, 251,  34, 242, 193, 238, 210, 144,  12, 191, 179, 162, 241,
            81,  51, 145, 235, 249,  14, 239, 107,  49, 192, 214,  31, 181, 199, 106, 157,
            184,  84, 204, 176, 115, 121,  50,  45, 127,   4, 150, 254, 138, 236, 205,  93,
            222, 114,  67,  29,  24,  72, 243, 141, 128, 195,  78,  66, 215,  61, 156, 180
    };

    int p[512];

    double3 center = double3(0.5, 0.5, 0.5);


};



