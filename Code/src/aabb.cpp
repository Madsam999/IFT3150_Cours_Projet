#include "aabb.h"

// @@@@@@ VOTRE CODE ICI
// Implémenter l'intersection d'un rayon avec un AABB dans l'intervalle décrit.
bool AABB::intersect(Ray ray, double t_min, double t_max) { return true; };

// class AABB {
// public:
//   double3 min;
//   double3 max;
//
//   // Calcul l'intersection d'un rayon avec un AABB qui respecte l'intervalle
//   de
//   // profondeur décrit.
//   bool intersect(Ray ray, double t_min, double t_max);
// };
// @@@@@@ VOTRE CODE ICI
// Implémenter la fonction qui permet de trouver les 8 coins de notre AABB.
std::vector<double3> retrieve_corners(AABB aabb) {
  return std::vector<double3>{double3{aabb.min.x, aabb.min.y, aabb.min.z},
                              double3{aabb.min.x, aabb.min.y, aabb.max.z},
                              double3{aabb.min.x, aabb.max.y, aabb.min.z},
                              double3{aabb.min.x, aabb.max.y, aabb.max.z},
                              double3{aabb.max.x, aabb.min.y, aabb.min.z},
                              double3{aabb.max.x, aabb.min.y, aabb.max.z},
                              double3{aabb.max.x, aabb.max.y, aabb.min.z},
                              double3{aabb.max.x, aabb.max.y, aabb.max.z}};
};

// @@@@@@ VOTRE CODE ICI
// Implémenter la fonction afin de créer un AABB qui englobe tous les points.
AABB construct_aabb(std::vector<double3> points) {
  double3 min = double3{DBL_MAX, DBL_MAX, DBL_MAX};
  double3 max = double3{-DBL_MAX, -DBL_MAX, -DBL_MAX};

  // We find the min and max of each component of the points
  for (const double3 point : points) {
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
  }

  return AABB{min, max};
};

AABB combine(AABB a, AABB b) {
  return AABB{min(a.min, b.min), max(a.max, b.max)};
};

bool compare(AABB a, AABB b, int axis) { return a.min[axis] < b.min[axis]; };
