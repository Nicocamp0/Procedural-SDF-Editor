/// =====================================================
/// Fonctions de distance signée (SDF)
/// VENTADOUX NICOLAS - BERTON UGO - SMATI LUCAS
/// Géométrie Numérique - Université de Strasbourg - 2026
/// =====================================================

// INCLUDE POUR AVOIR ACCES AUX RESSOURCES NECESSAIRES
#include "../include/SDF.h"
#include <cmath>
#include <algorithm>


/// Sdf du Cône

float SDF_Cone(const Eigen::Vector3f& point, const Eigen::Vector2f& direction, float height)
{
    // q = point de la base du cône en 2D
    float denom = std::max(direction.y(), 1e-6f);       // sécurité pour éviter division par 0
    Eigen::Vector2f q = height * Eigen::Vector2f(direction.x() / denom, -1.0f);

    // projection du point en 2D (rayon, height)
    float r = std::sqrt(point.x()*point.x() + point.z()*point.z());
    Eigen::Vector2f w(r, point.y());

    // projection sur le segment incliné
    float t1 = std::clamp(w.dot(q) / q.dot(q), 0.0f, 1.0f);
    Eigen::Vector2f a = w - q * t1;

    // projection sur la base
    float t2 = std::clamp(w.x() / q.x(), 0.0f, 1.0f);
    Eigen::Vector2f b = w - Eigen::Vector2f(q.x() * t2, q.y());

    float k = (q.y() < 0.0f) ? -1.0f : 1.0f;

    float d = std::min(a.dot(a), b.dot(b));

    float s = std::max(
        k * (w.x() * q.y() - w.y() * q.x()),
        k * (w.y() - q.y())
    );

    return std::sqrt(d) * ((s < 0.0f) ? -1.0f : 1.0f);
}


/// SdF du Link 

float SDF_Link(const Eigen::Vector3f& point, float half_length, float major_radius, float minor_radius)
{
  Eigen::Vector3f q = Eigen::Vector3f(point.x(), std::max(std::abs(point.y())-half_length, 0.0f), point.z());
  float k = Eigen::Vector2f(q.x(),q.y()).norm();
  return (Eigen::Vector2f(k-major_radius,q.z())).norm() - minor_radius;
}



/// SdF du Torus

float SDF_Torus(const Eigen::Vector3f& point, const Eigen::Vector2f& radii){
    Eigen::Vector2f pxz(point.x(), point.z());
    float len_xz = pxz.norm();

    Eigen::Vector2f q(len_xz - radii.x(), point.y());

    return q.norm() - radii.y();
}



/// SdF de la Sphère

float SDF_Sphere(const Eigen::Vector3f& point, float radius){
    return point.norm() - radius;
}


/// SdF de la Boîte

float SDF_Box(const Eigen::Vector3f& point, const Eigen::Vector3f& half_lengths){
    Eigen::Vector3f q = point.cwiseAbs() - half_lengths;

    float outside = q.cwiseMax(0.0f).norm();
    float inside = std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.0f);

    return outside + inside;
}


/// SdF de l'Octaèdre

float SDF_Octahedron(const Eigen::Vector3f& point, float scale){
    Eigen::Vector3f p = point.cwiseAbs();
    float m = p.x() + p.y() + p.z() - scale;

    Eigen::Vector3f q;
    if (3.0f * p.x() < m)
        q = Eigen::Vector3f(p.x(), p.y(), p.z());
    else if (3.0f * p.y() < m)
        q = Eigen::Vector3f(p.y(), p.z(), p.x());
    else if (3.0f * p.z() < m)
        q = Eigen::Vector3f(p.z(), p.x(), p.y());
    else
        return m * 0.57735027f;

    float k = std::clamp(0.5f * (q.z() - q.y() + scale), 0.0f, scale);
    return Eigen::Vector3f(q.x(), q.y() - scale + k, q.z() - k).norm();
}

/// SdF du Cylindre

float SDF_Cylinder(const Eigen::Vector3f& point, float radius, float half_height)
{
    Eigen::Vector2f d;
    d << std::sqrt(point.x() * point.x() + point.z() * point.z()), point.y();
    d = d.cwiseAbs() - Eigen::Vector2f(radius, half_height);

    float inside = std::min(std::max(d.x(), d.y()), 0.0f);
    float outside = d.cwiseMax(0.0f).norm();

    return inside + outside;
}


/// Opérations booléennes entre deux SDF

float opUnion(float a, float b)
{
    return std::min(a, b);
}

float opIntersection(float a , float b)
{
    return std::max(a, b);
}

float opDifference(float a, float b)
{
    return std::max(a, -b);
}