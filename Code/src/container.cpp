#include "container.h"
#include <algorithm>

// @@@@@@ VOTRE CODE ICI
// - Parcourir l'arbre DEPTH FIRST SEARCH selon les conditions suivantes:
// 		- S'il s'agit d'une feuille, faites l'intersection avec la
// géométrie.
//		- Sinon, il s'agit d'un noeud altérieur.
//			- Faites l'intersection du rayon avec le AABB gauche et
// droite.
//				- S'il y a intersection, ajouter le noeud à ceux
// à visiter.
// - Retourner l'intersection avec la profondeur maximale la plus PETITE.
bool BVH::intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
  if (!root) {
    return false;
  }
  std::vector<BVHNode *> toVisit;
  toVisit.push_back(root);
  bool isHit = false;
  while (!toVisit.empty()) {
    // Get a ptr to the last node in the stack
    BVHNode *node = toVisit.back();
    // Pop the last node from the stack
    // it does not return a value :(
    toVisit.pop_back();
    // check if it is a leaf
    if (node->left == nullptr && node->right == nullptr) {
      // intersect with the object
      if (objects[node->idx]->intersect(ray, t_min, t_max, hit)) {
        t_max = hit->depth;
        isHit = true;
      }
    } else {
      // Check if the ray intersect with the left AABB
      if (node->left->aabb.intersect(ray, t_min, t_max)) {
        // Add the left node to the stack
        toVisit.push_back(node->left);
      }
      // Check if the ray intersect with the right AABB
      if (node->right->aabb.intersect(ray, t_min, t_max)) {
        // Add the right node to the stack
        toVisit.push_back(node->right);
      }
    }
  }
  return isHit;
}
// @@@@@@ VOTRE CODE ICI
// - Parcourir tous les objets
// 		- Détecter l'intersection avec l'AABB
//			- Si intersection, détecter l'intersection avec la
// géométrie.
//				- Si intersection, mettre à jour les paramètres.
// - Retourner l'intersection avec la profondeur maximale la plus PETITE.
bool Naive::intersect(Ray ray, double t_min, double t_max, Intersection *hit) {
  bool isHit = false;

/*
   for (size_t i = 0; i < objects.size(); ++i) {
    if (aabbs[i].intersect(ray, t_min, t_max)) {
      if (objects[i]->intersect(ray, t_min, t_max, hit)) {
        t_max = hit->depth;
        isHit = true;
      }
    }
  }
  */




  //std::cout << "Naive intersect" << std::endl;
  for(size_t i = 0; i < mediums.size(); i++) {
      //std::cout << "Medium intersect" << std::endl;
      if(mediums[i]->intersect(ray, t_min, t_max, hit)){
          t_max = hit->depth;
          isHit = true;
      }
  }



  return isHit;
}
