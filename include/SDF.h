#ifndef MODULE_SDF_H
#define MODULE_SDF_H

// Include de Eigen pour pouvoir s'en servir
#include <Eigen/Core>


/// =====================================================
/// Fonctions de distance signée (SDF)
/// VENTADOUX NICOLAS - BERTON UGO - SMATI LUCAS
/// Géométrie Numérique - Université de Strasbourg - 2026
/// =====================================================
///
/// Convention :
/// - Toutes les primitives sont centrées en (0,0,0)
/// - d < 0 : point à l’intérieur
/// - d = 0 : point sur la surface
/// - d > 0 : point à l’extérieur
///
/// Les points sont passés en référence constante pour éviter les copies
/// =====================================================



/// A NOTER : A date du 15/04/26, seulement 6 primitives sont implémentées


/// SdF du CÔNE 

/// @brief Fonction de distance signée d’un cône
/// @param point Point de l’espace à tester
/// @param direction Vecteur (sin(angle), cos(angle)) définissant l’orientation
/// @param height Hauteur du cône
/// @return Distance signée au cône 
float SDF_Cone(const Eigen::Vector3f& point, const Eigen::Vector2f& direction, float height);


/// SdF d'un Maillon (Link) 

/// @brief Fonction de distance signée d’un maillon de chaîne
/// @param point Point de l’espace à tester
/// @param half_length Demi-longueur de la partie rectiligne centrale
/// @param major_radius Rayon principal (distance au centre)
/// @param minor_radius Épaisseur du tube
/// @return Distance signée au maillon
float SDF_Link(const Eigen::Vector3f& point, float half_length, float major_radius, float minor_radius);


/// SdF du Torus

/// @brief Fonction de distance signée d’un tore
/// @param point Point de l’espace à tester
/// @param radii Vecteur (rayon majeur, rayon mineur) => Radii est le pluriel de radius
/// @return Distance signée au tore
float SDF_Torus(const Eigen::Vector3f& point, const Eigen::Vector2f& radii);

/// SdF de la sphère 

/// @brief Fonction distance signée pour la primitive Sphère
/// @param point Point de l'espace à tester
/// @param radius Rayon de la sphère
/// @return Valeur disant où est le point de l'espace p/r à la primitive construite
float SDF_Sphere(const Eigen::Vector3f& point, float radius);


/// ATTENTION 
/// A partir d'ici ce sont les SdF de Nicolas que j'ai reprises (UGO)
// Il faudra les tester pour vérifier qu'elles sont correctes


/// Sdf de la boîte 

/// @brief Fonction de distance signée d’une boîte axis-alignée
/// @param point Point de l’espace à tester
/// @param half_lengths Demi-dimensions de la boîte (x, y, z)
/// @return Distance signée à la boîte
float SDF_Box(const Eigen::Vector3f& point, const Eigen::Vector3f& half_lengths);


/// SdF de l'octaèdre

/// @brief Fonction de distance signée d’un octaèdre (norme L1)
/// @param point Point de l’espace à tester
/// @param scale Échelle de l’octaèdre (rayon L1)
/// @return Distance signée à l’octaèdre
float SDF_Octahedron(const Eigen::Vector3f& point, float scale);

/// SdF du cylindre

/// @brief Fonction de distance signée d’un cylindre
/// @param point Point de l’espace à tester
/// @param radius Rayon du cylindre
/// @param half_height Demi-hauteur du cylindre
/// @return Distance signée au cylindre
float SDF_Cylinder(const Eigen::Vector3f& point, float radius, float half_height);



/// Opérations booléennes entre deux SDF
float opUnion(float a, float b);
float opIntersection(float a, float b);
float opDifference(float a, float b);


#endif

