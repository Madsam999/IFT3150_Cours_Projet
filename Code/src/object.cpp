#include "object.h"
#include "aabb.h"
#include "basic.h"
#include <cstdlib>
#include <functional>

// Fonction retournant soit la valeur v0 ou v1 selon le signe.
int rsign(double value, double v0, double v1) {
  return (int(std::signbit(value)) * (v1 - v0)) + v0;
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de trouver l'intersection d'une
// sphère.
//
// Référez-vous au PDF pour la paramétrisation des coordonnées UV.
//
// Pour plus de d'informations sur la géométrie, référez-vous à la classe
// object.h.

bool Sphere::local_intersect(Ray ray, double t_min, double t_max,
                             Intersection *hit) {

  // TODO: Si la sphère est pas opaque et que le rayon provient de l'interieur
  // ou sur la sphère.

  // Si le rayon provient de l'interieur ou sur la sphère.
  if (pow(ray.origin.x, 2) + pow(ray.origin.y, 2) + pow(ray.origin.z, 2) <=
      pow(radius, 2)) {
    // TODO: Si la sphère est pas opaque
  }

  double A = dot(ray.direction, ray.direction);
  double B = 2 * dot(ray.direction, ray.origin);
  double C = dot(ray.origin, ray.origin) - pow(radius, 2);

  double discriminant = B * B - 4 * A * C;

  if (discriminant > 0) {
    double firstDepth = (-B + sqrt(discriminant)) / (2 * A);
    double secondDepth = (-B - sqrt(discriminant)) / (2 * A);

    double minDepth = std::min(firstDepth, secondDepth);
    double maxDepth = std::max(firstDepth, secondDepth);

    if (minDepth < t_min || maxDepth > t_max) {
      return false;
    }

    if (maxDepth < 0) {
      return false;
    } else if (minDepth < 0) {
      hit->depth = maxDepth;
    } else {
      hit->depth = minDepth;
    }
  } else if (discriminant < 0) {
    return false;
  } else if (discriminant == 0) {
    hit->depth = -B / (2 * A);
  } else {
    return false;
  }
  hit->position = ray.origin + hit->depth * ray.direction;
  hit->normal = normalize(hit->position); // car centré à l'origine

  double theta = acos(hit->position.y / radius);
  double phi = atan2(hit->position.x, hit->position.z);

  if (phi < 0.0) {
    phi += 2 * PI;
  }

  double u = phi / (2 * PI);
  double v = (theta) / PI;

  hit->uv = double2(u, v);

  return true;
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de calculer le AABB pour la
// sphère. Il faut que le AABB englobe minimalement notre objet à moins que
// l'énoncé prononce le contraire (comme ici).
AABB Sphere::compute_aabb() {
  return AABB{
      double3(-radius, -radius, -radius),
      double3(radius, radius, radius),
  };
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de trouver l'intersection
// avec un quad (rectangle).
//
// Référez-vous au PDF pour la paramétrisation des coordonnées UV.
//
// Pour plus de d'informations sur la géométrie, référez-vous à la classe
// object.h.
bool Quad::local_intersect(Ray ray, double t_min, double t_max,
                           Intersection *hit) {
  // TODO: Si le rayon est parallèle au plan, si le rayon a une origine sur le
  // plan.
  //
  // equation implicite du plan
  double3 normal = double3(0, 0, 1);
  if (dot(ray.direction, normal) == 0) {
    return false;
  }
  if (dot(ray.origin, normal) < 0) {
    normal = -normal;
  }

  // If the ray is parallel to the plane
  double t;
  t = -dot(ray.origin, normal) / dot(ray.direction, normal);

  // If the ray is behind the camera
  if (t < t_min || t > t_max) {
    return false;
  }

  double3 p = ray.origin + t * ray.direction;

  double h = this->half_size;

  // If the intersection is outside the quad bounds
  if (p.x < -h || p.x > h || p.y < -h || p.y > h) {
    return false;
  }
  hit->depth = t;
  hit->position = p;
  hit->normal = normalize(normal);

  // the image (0,0) is at the top left whereas the (0,0) in uv is at the
  // bottom left so we need to modify v accordingly.
  double u = (p.x + half_size) / (2.0 * half_size);
  double v = (p.y - half_size) / (-2.0 * half_size);

  hit->uv = double2(u, v);

  return true;
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de calculer le AABB pour le
// quad (rectangle). Il faut que le AABB englobe minimalement notre objet à
// moins que l'énoncé prononce le contraire.
AABB Quad::compute_aabb() {
  return AABB{
      double3(-half_size, -half_size, 0),
      double3(half_size, half_size, 0),
  };
  // return Object::compute_aabb();
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de trouver l'intersection
// avec un cylindre.
//
// Référez-vous au PDF pour la paramétrisation des coordonnées UV.
//
// Pour plus de d'informations sur la géométrie, référez-vous à la classe
// object.h.
bool Cylinder::local_intersect(Ray ray, double t_min, double t_max,
                               Intersection *hit) {
  // if the ray is parallel to the cylinder
  if (ray.direction == double3(0, 0, 0)) {
    return false;
  }

  double a = pow(ray.direction.x, 2) + pow(ray.direction.z, 2);
  double b =
      2 * (ray.direction.x * (ray.origin.x) + ray.direction.z * (ray.origin.z));
  double c = pow(ray.origin.x, 2) + pow(ray.origin.z, 2) - pow(radius, 2);

  double discriminant = pow(b, 2) - 4 * a * c;

  if (discriminant < 0) {
    return false;
  }

  double firstDepth = (-b + sqrt(discriminant)) / (2 * a);
  double secondDepth = (-b - sqrt(discriminant)) / (2 * a);

  double t;

  if (firstDepth < secondDepth && firstDepth >= t_min && firstDepth <= t_max) {
    t = firstDepth;
  } else if (secondDepth >= t_min && secondDepth <= t_max) {
    double temp = firstDepth;
    firstDepth = secondDepth;
    secondDepth = temp;
    t = firstDepth;
  } else {
    return false;
  }

  double3 p = ray.origin + t * ray.direction;

  // Check if the intersection is inside, outside or on the cylinder

  if (p.y > this->half_height || p.y < -this->half_height) {
    t = secondDepth;
    p = ray.origin + t * ray.direction;
    if (p.y > this->half_height || p.y < -this->half_height) {
      return false;
    }
    // The ray is inside the cylinder
    hit->normal = normalize(double3(p.x, 0, p.z) * -1);
  } else {
    // The ray is outside the cylinder
    hit->normal = normalize(double3(p.x, 0, p.z));
  }

  hit->depth = t;
  hit->position = p;

  auto x = p.x;
  auto y = p.y;
  auto z = p.z;

  auto v = (p.y - half_height) / (-2 * half_height);

  double phi = atan2(x, z);

  if (phi < 0.0) {
    phi += 2 * PI;
  }

  auto u = phi / (2 * PI) + 0.75;

  hit->uv = double2(u, v);

  return true;
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de calculer le AABB pour le
// cylindre. Il faut que le AABB englobe minimalement notre objet à moins que
// l'énoncé prononce le contraire (comme ici).
AABB Cylinder::compute_aabb() {
  return AABB{
      double3(-radius, -half_height, -radius),
      double3(radius, half_height, radius),
  };
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de trouver l'intersection
// avec un mesh.
//
// Référez-vous au PDF pour la paramétrisation pour les coordonnées UV.
//
// Pour plus de d'informations sur la géométrie, référez-vous à la classe
// object.h.
//
bool Mesh::local_intersect(Ray ray, double t_min, double t_max,
                           Intersection *hit) {
  bool has_hit = false;
  for (Triangle const &tri : triangles) {
    has_hit |= intersect_triangle(ray, t_min, t_max, tri, hit);
  }
  return has_hit;
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de trouver l'intersection
// avec un triangle. S'il y a intersection, remplissez hit avec l'information
// sur la normale et les coordonnées texture.
bool Mesh::intersect_triangle(Ray ray, double t_min, double t_max,
                              Triangle const tri, Intersection *hit) {
  // Extrait chaque position de sommet des données du maillage.
  double3 const &p0 =
      positions[tri[0].pi]; // ou Sommet A (Pour faciliter les explications)
  double3 const &p1 = positions[tri[1].pi]; // ou Sommet B
  double3 const &p2 = positions[tri[2].pi]; // ou Sommet C

  double3 n1 = normals[tri[0].ni];
  // Triangle en question. Respectez la convention suivante pour vos
  // variables.
  //
  //     A
  //    / \
  //   / \
  //  B --> C
  //
  // Respectez la règle de la main droite pour la normale.

  // @@@@@@ VOTRE CODE ICI
  // Décidez si le rayon intersecte le triangle (p0,p1,p2).
  // Si c'est le cas, remplissez la structure hit avec les informations
  // de l'intersection et renvoyez true.
  // Pour plus de d'informations sur la géométrie, référez-vous à la classe
  // dans object.hpp.
  //
  // NOTE : hit.depth est la profondeur de l'intersection actuellement la plus
  // proche, donc n'acceptez pas les intersections qui occurent plus loin que
  // cette valeur.
  //

  double3 e1 = p1 - p0;
  double3 e2 = p2 - p0;
  double3 q = cross(ray.direction, e2);
  double a = dot(e1, q);

  if (a > -EPSILON && a < EPSILON) {
    return false;
  }

  double f = 1 / a;
  double3 s = ray.origin - p0;
  double u = f * dot(s, q);

  if (u < 0) {
    return false;
  }

  double3 r = cross(s, e1);

  double v = f * dot(ray.direction, r);

  if (v < 0 || u + v > 1) {
    return false;
  }

  double t = f * dot(e2, r);

  if (t < t_min || t > t_max || t < 0) {
    return false;
  }

  // Remove the backface culling
  if (dot(ray.direction, cross(e1, e2)) > 0) {
    return false;
  }

  hit->depth = t;
  hit->position = ray.origin + t * ray.direction;
  // hit->normal = normalize(cross(e1, e2));

  // Basé sur le code de Caio au début;
  double2 texCoordA = tex_coords[tri[0].ti];
  double2 texCoordB = tex_coords[tri[1].ti];
  double2 texCoordC = tex_coords[tri[2].ti];

  auto normA = normals[tri[0].ni];
  auto normB = normals[tri[1].ni];
  auto normC = normals[tri[2].ni];

  double3 v0 = p1 - p0;
  double3 v1 = p2 - p0;
  double3 v2 = hit->position - p0;

  double d00 = dot(v0, v0);
  double d01 = dot(v0, v1);
  double d11 = dot(v1, v1);
  double d20 = dot(v2, v0);
  double d21 = dot(v2, v1);
  double denom = d00 * d11 - d01 * d01;

  auto baryV = (d11 * d20 - d01 * d21) / denom;
  auto baryW = (d00 * d21 - d01 * d20) / denom;
  auto baryU = 1.0 - baryV - baryW;

  auto texU = baryU * texCoordA.x + baryV * texCoordB.x + baryW * texCoordC.x;
  auto texV = baryU * texCoordA.y + baryV * texCoordB.y + baryW * texCoordC.y;

  auto normX = baryU * normA.x + baryV * normB.x + baryW * normC.x;
  auto normY = baryU * normA.y + baryV * normB.y + baryW * normC.y;
  auto normZ = baryU * normA.z + baryV * normB.z + baryW * normC.z;

  hit->uv = double2(texU, texV);
  hit->normal = normalize(double3(normX, normY, normZ));

  return true;
}

// @@@@@@ VOTRE CODE ICI
// Occupez-vous de compléter cette fonction afin de calculer le AABB pour le
// Mesh. Il faut que le AABB englobe minimalement notre objet à moins que
// l'énoncé prononce le contraire.
AABB Mesh::compute_aabb() {
  double3 min = double3(0, 0, 0);
  double3 max = double3(0, 0, 0);

  for (double3 const &position : positions) {
    min = double3(std::min(min.x, position.x), std::min(min.y, position.y),
                  std::min(min.z, position.z));
    max = double3(std::max(max.x, position.x), std::max(max.y, position.y),
                  std::max(max.z, position.z));
  }

  return AABB{min, max};
}
