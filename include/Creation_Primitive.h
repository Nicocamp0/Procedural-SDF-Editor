#ifndef CREATION_PRIMITIVE_H
#define CREATION_PRIMITIVE_H

// Include de Eigen pour pouvoir s'en servir
#include <Eigen/Core>
#include "Mesh.h"           // pour séparer responsabilités et mettre les données à un endroit séparé


/// =====================================================
/// Fonctions de création de primitives (Cône, Tore, Link, ...)
/// VENTADOUX NICOLAS - BERTON UGO - SMATI LUCAS
/// Géométrie Numérique - Université de Strasbourg - 2026
/// =====================================================


/// Création Cône

/// @brief Fonction pour créer cône dans éditeur
/// @param mesh Mesh structure à remplir
/// @param height Hauteur du cône 
/// @param angle Angle pour la pointe du cône
/// @param grid_size Résolution de la grille
void create_cone(MeshData& mesh, float height = 2.0f, float angle = 3.14159265f / 4.0f, int grid_size = 20);


/// Création Link

/// @brief 
/// @param mesh Mesh structure à remplir
/// @param half_length Demi-longueur de la partie centrale rectiligne du maillon
/// @param inner_radius Rayon du tube (épaisseur)
/// @param major_radius Distance du centre au centre du tube
/// @param grid_size Résolution de la grille
void create_link(MeshData& mesh, float half_length = 2.0f, float inner_radius = 1.0f, float major_radius = 4.0f, int grid_size = 20);


/// Création Torus

/// @brief Fonction pour créer torus dans éditeur
/// @param mesh Structure pour stocker le mesh
/// @param inner_radius Rayon interne au torus <=> épaisseur du torus
/// @param major_radius Rayon du torus du centre à la center line
/// @param grid_size Résolution de la grille
void create_torus(MeshData& mesh, float inner_radius = 1.0f, float major_radius = 2.0f, int grid_size = 20);



/// Création Sphère

/// @brief Fonction pour créer maillage sphère dans éditeur
/// @param mesh Mesh structure à remplir
/// @param radius Rayon de la sphère
/// @param grid_size Résolution de la grille
void create_sphere(MeshData& mesh, float radius = 1.0f, int grid_size = 30);


/// Création Boîte


/// @brief Fonction pour créer maillage Boîte
/// @param mesh Mesh structure à remplir
/// @param half_dimensions Demi-dimensions (x,y,z) pour création de la boîte 
/// @param grid_size Résolution de la grille
void create_box(MeshData& mesh, const Eigen::Vector3f& half_dimensions, int grid_size = 30);

/// Création Octaèdre

/// @brief Fonction pour créer un maillage octaèdre
/// @param mesh Mesh structure à remplir
/// @param size Echelle de l'octaèdre
/// @param grid_size Résolution de la grille
void create_octahedron(MeshData& mesh, float size = 0.5f, int grid_size = 30);

/// Création Cube
void create_cube(MeshData& mesh, float half_size = 0.5f, int grid_size = 30);

/// Création Cylindre
void create_cylinder(MeshData& mesh, float radius = 0.5f, float half_height = 0.75f, int grid_size = 30);

/// Création du maillage résultant d'une union entre deux primitives
void create_union_mesh(const MeshData& a, const MeshData& b, MeshData& result, int grid_size = 50);
void create_intersection_mesh(const MeshData& a, const MeshData& b, MeshData& result, int grid_size = 50);
void create_difference_mesh(const MeshData& a, const MeshData& b, MeshData& result, int grid_size = 50);

#endif