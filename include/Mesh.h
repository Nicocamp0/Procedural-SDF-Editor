#ifndef MESH_H
#define MESH_H

#include <Eigen/Core>           //pour avoir les différents types nécessaires
#include <memory>

/// =====================================================
/// Déclaration d'une structure de maillage pour les primitives
/// VENTADOUX NICOLAS - BERTON UGO - SMATI LUCAS
/// Géométrie Numérique - Université de Strasbourg - 2026
/// =====================================================

enum class PrimitiveType
{
    None,
    Sphere,
    Box,
    Torus,
    Link,
    Cone,
    Octahedron,
    Cylinder,
    UnionResult,
    IntersectionResult,
    DifferenceResult
};

// Structure données d'un Mesh
struct MeshData {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd V_rest;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    std::shared_ptr<MeshData> left=nullptr;
    std::shared_ptr<MeshData> right=nullptr;

    int data_id = -1;

    // Type de primitive
    PrimitiveType type = PrimitiveType::None;

    // Paramètres géométriques
    float radius = 1.0f;   // sphère / cylindre
    float height = 1.0f;   // cylindre / cône

    Eigen::Vector3f half_dimensions = Eigen::Vector3f(0.5f, 0.5f, 0.5f); // boîte

    float inner_radius = 1.0f;   // tore / link
    float major_radius = 2.0f;   // tore / link
    float half_length = 2.0f;    // link

    float angle = 3.14159265f / 4.0f; // cône

    float octa_size = 0.5f;      // octaèdre
    Eigen::MatrixXd C; // couleur par sommet
    Eigen::RowVector3d color = Eigen::RowVector3d(1.0, 1.0, 1.0);
};

#endif