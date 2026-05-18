/// =====================================================
/// Fonctions de création de primitives (Cône, Tore, Link, ...)
/// VENTADOUX NICOLAS - BERTON UGO - SMATI LUCAS
/// Géométrie Numérique - Université de Strasbourg - 2026
/// =====================================================


// INCLUDE POUR AVOIR ACCES AUX RESSOURCES NECESSAIRES
#include "../include/Creation_Primitive.h"
#include "../include/SDF.h"
#include <igl/marching_cubes.h>
#include <cmath>
#include <algorithm>

/// Création de la grille 3D d'échantillonnage

static void create_grid(Eigen::MatrixXd &grid, float min, float max, int size) {
    grid.resize(size * size * size, 3);
    float step = (max - min) / (size - 1);
    int idx = 0;
    for (int i = 0; i < size; ++i) {
        for (int j = 0; j < size; ++j) {
            for (int k = 0; k < size; ++k) {
                grid(idx, 0) = min + i * step;
                grid(idx, 1) = min + j * step;
                grid(idx, 2) = min + k * step;
                idx++;
            }
        }
    }
}

/// Fonctions utilitaires pour les opérations SDF

static float evaluate_primitive_sdf_local(const MeshData& mesh, const Eigen::Vector3f& p_local)
{
    switch (mesh.type)
    {
        case PrimitiveType::Sphere:
            return SDF_Sphere(p_local, mesh.radius);

        case PrimitiveType::Box:
            return SDF_Box(p_local, mesh.half_dimensions);

        case PrimitiveType::Torus:
        {
            Eigen::Vector2f t(mesh.major_radius, mesh.inner_radius);
            return SDF_Torus(p_local, t);
        }

        case PrimitiveType::Link:
            return SDF_Link(p_local, mesh.half_length, mesh.major_radius, mesh.inner_radius);

        case PrimitiveType::Cone:
        {
            float sin_a = std::sin(mesh.angle);
            float cos_a = std::cos(mesh.angle);
            Eigen::Vector2f c(sin_a, cos_a);
            return SDF_Cone(p_local, c, mesh.height);
        }

        case PrimitiveType::Octahedron:
            return SDF_Octahedron(p_local, mesh.octa_size);

        case PrimitiveType::Cylinder:
            return SDF_Cylinder(p_local, mesh.radius, mesh.height);

        default:
            return 1e9f;
    }
}

static float evaluate_primitive_sdf_world(const MeshData& mesh, const Eigen::Vector3f& p_world)
{
    if (mesh.type == PrimitiveType::UnionResult && mesh.left && mesh.right)
    {
        float da = evaluate_primitive_sdf_world(*mesh.left, p_world);
        float db = evaluate_primitive_sdf_world(*mesh.right, p_world);
        return opUnion(da, db);
    }

    if (mesh.type == PrimitiveType::IntersectionResult && mesh.left && mesh.right)
    {
        float da = evaluate_primitive_sdf_world(*mesh.left, p_world);
        float db = evaluate_primitive_sdf_world(*mesh.right, p_world);
        return opIntersection(da, db);
    }

    if (mesh.type == PrimitiveType::DifferenceResult && mesh.left && mesh.right)
    {
        float da = evaluate_primitive_sdf_world(*mesh.left, p_world);
        float db = evaluate_primitive_sdf_world(*mesh.right, p_world);
        return opDifference(da, db);
    }

    Eigen::Matrix4f Tinv = mesh.T.inverse();

    Eigen::Vector4f p4;
    p4 << p_world.x(), p_world.y(), p_world.z(), 1.0f;

    Eigen::Vector4f p_local4 = Tinv * p4;
    Eigen::Vector3f p_local = p_local4.head<3>();

    return evaluate_primitive_sdf_local(mesh, p_local);
}



/// Création Cône

void create_cone(MeshData& mesh, float height, float angle, int grid_size)
{
    Eigen::MatrixXd grid;

    create_grid(grid, -3.0f * height, 3.0f * height, grid_size);

    float sin_a = std::sin(angle);
    float cos_a = std::cos(angle);
    Eigen::Vector2f c(sin_a, cos_a);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();
        sdf_values(i) = SDF_Cone(p, c, height);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    // Appel au marching cube pour construire primitive
    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();

    mesh.type = PrimitiveType::Cone;
    mesh.height = height;
    mesh.angle = angle;
}



/// Création Link

void create_link(MeshData& mesh, float half_length, float inner_radius, float major_radius, int grid_size)
{
    Eigen::MatrixXd grid;

    float extent = major_radius + inner_radius + 2.0f * half_length;
    create_grid(grid, -extent, extent, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();

        // ATTENTION : signature SDF_Link(point, half_length, major_radius, minor_radius)
        sdf_values(i) = SDF_Link(p, half_length, major_radius, inner_radius);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    // Appel au marching cube pour construire primitive
    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();

    mesh.type = PrimitiveType::Link;
    mesh.half_length = half_length;
    mesh.inner_radius = inner_radius;
    mesh.major_radius = major_radius;
}

/// Création Torus

void create_torus(MeshData& mesh, float inner_radius, float major_radius, int grid_size)
{
    Eigen::MatrixXd grid;

    float extent = major_radius + inner_radius;
    create_grid(grid, -extent, extent, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    Eigen::Vector2f t(major_radius, inner_radius);

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();
        sdf_values(i) = SDF_Torus(p, t);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    // Appel au marching cube pour construire primitive
    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();

    mesh.type = PrimitiveType::Torus;
    mesh.inner_radius = inner_radius;
    mesh.major_radius = major_radius;
}


/// Création Sphère

void create_sphere(MeshData& mesh, float radius, int grid_size)
{
    Eigen::MatrixXd grid;

    create_grid(grid, -radius, radius, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();
        sdf_values(i) = SDF_Sphere(p, radius);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();

    mesh.type = PrimitiveType::Sphere;
    mesh.radius = radius;
}


/// Création Boîte

void create_box(MeshData& mesh, const Eigen::Vector3f& half_dimensions, int grid_size)
{
    Eigen::MatrixXd grid;

    float extent = half_dimensions.maxCoeff() * 1.5f;           // récupère dimension max pour avoir une boîte propre
    create_grid(grid, -extent, extent, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();
        sdf_values(i) = SDF_Box(p, half_dimensions);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();

    mesh.type = PrimitiveType::Box;
    mesh.half_dimensions = half_dimensions;
}


/// Création Cube

void create_cube(MeshData& mesh, float half_size, int grid_size)
{
    create_box(mesh, Eigen::Vector3f(half_size, half_size, half_size), grid_size);
}


/// Création Octaèdre 

void create_octahedron(MeshData& mesh, float size, int grid_size)
{
    Eigen::MatrixXd grid;

    create_grid(grid, -1.5f * size, 1.5f * size, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();
        sdf_values(i) = SDF_Octahedron(p, size);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();

    mesh.type = PrimitiveType::Octahedron;
    mesh.octa_size = size;
}


/// Création Cylindre 

void create_cylinder(MeshData& mesh, float radius, float half_height, int grid_size)
{
    Eigen::MatrixXd grid;

    float bound = 1.5f * std::max(radius, half_height);
    create_grid(grid, -bound, bound, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();
        sdf_values(i) = SDF_Cylinder(p, radius, half_height);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();

    mesh.type = PrimitiveType::Cylinder;
    mesh.radius = radius;
    mesh.height = half_height;
}


///Création du mélange des couleurs

static Eigen::RowVector3d evaluate_color_world(const MeshData& mesh, const Eigen::Vector3f& p_world)
{
    if (mesh.left && mesh.right)
    {
        float da = evaluate_primitive_sdf_world(*mesh.left, p_world);
        float db = evaluate_primitive_sdf_world(*mesh.right, p_world);

        Eigen::RowVector3d colorA = evaluate_color_world(*mesh.left, p_world);
        Eigen::RowVector3d colorB = evaluate_color_world(*mesh.right, p_world);

        float blend = 0.5f;

        if (mesh.type == PrimitiveType::UnionResult)
        {
            float t = 0.5f + (db - da) / (2.0f * blend);
            t = std::clamp(t, 0.0f, 1.0f);

            return t * colorA + (1.0f - t) * colorB;
        }

        if (mesh.type == PrimitiveType::IntersectionResult)
        {
            float t = 0.5f + (da - db) / (2.0f * blend);
            t = std::clamp(t, 0.0f, 1.0f);

            return t * colorA + (1.0f - t) * colorB;
        }

        if (mesh.type == PrimitiveType::DifferenceResult)
        {
            float t = 0.5f + (db - da) / (2.0f * blend);
            t = std::clamp(t, 0.0f, 1.0f);

            return t * colorA + (1.0f - t) * colorB;
        }
    }

    return mesh.color;
}


/// Création du maillage résultant d'une union entre deux primitives

void create_union_mesh(const MeshData& a, const MeshData& b, MeshData& result, int grid_size)
{
    Eigen::MatrixXd grid;

    // Version simple pour commencer :
    // on échantillonne dans un cube global fixe
    float min_bound = -3.0f;
    float max_bound =  3.0f;

    create_grid(grid, min_bound, max_bound, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();

        float da = evaluate_primitive_sdf_world(a, p);
        float db = evaluate_primitive_sdf_world(b, p);

        sdf_values(i) = opUnion(da, db);
    }

    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, result.V, result.F);

    result.C.resize(result.V.rows(), 3);
    result.type = PrimitiveType::UnionResult;
    result.left = std::make_shared<MeshData>(a);
    result.right = std::make_shared<MeshData>(b);

    for (int i = 0; i < result.V.rows(); ++i)
    {
        Eigen::Vector3f p = result.V.row(i).cast<float>();

        result.C.row(i) = evaluate_color_world(result, p);  
    }

    result.color = 0.5 * a.color + 0.5 * b.color;

    result.V_rest = result.V;
    result.T = Eigen::Matrix4f::Identity();
}

void create_intersection_mesh(const MeshData& a, const MeshData& b, MeshData& result, int grid_size)
{
    Eigen::MatrixXd grid;

    float min_bound = -3.0f;
    float max_bound =  3.0f;

    create_grid(grid, min_bound, max_bound, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();

        float da = evaluate_primitive_sdf_world(a, p);
        float db = evaluate_primitive_sdf_world(b, p);

        sdf_values(i) = opIntersection(da, db);
    }


    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;

    double iso_value = 0.0;

    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, result.V, result.F);


    result.type = PrimitiveType::IntersectionResult;
    result.left = std::make_shared<MeshData>(a);
    result.right = std::make_shared<MeshData>(b);

    result.C.resize(result.V.rows(), 3);

    for (int i = 0; i < result.V.rows(); ++i)
    {
        Eigen::Vector3f p = result.V.row(i).cast<float>();
        result.C.row(i) = evaluate_color_world(result, p);
    }

    result.color = 0.5 * a.color + 0.5 * b.color;

    result.V_rest = result.V;
    result.T = Eigen::Matrix4f::Identity();
}

void create_difference_mesh(const MeshData& a, const MeshData& b, MeshData& result, int grid_size)
{
    Eigen::MatrixXd grid;

    float min_bound = -3.0f;
    float max_bound =  3.0f;

    create_grid(grid, min_bound, max_bound, grid_size);

    Eigen::VectorXd sdf_values(grid.rows());

    for (int i = 0; i < grid.rows(); ++i)
    {
        Eigen::Vector3f p = grid.row(i).cast<float>();

        float da = evaluate_primitive_sdf_world(a, p);
        float db = evaluate_primitive_sdf_world(b, p);

        sdf_values(i) = opDifference(da, db);
    }


    Eigen::VectorXi dims(3);
    dims << grid_size, grid_size, grid_size;
    
    double iso_value = 0.0;

    igl::marching_cubes(sdf_values, grid, dims(0), dims(1), dims(2),
                        iso_value, result.V, result.F);


    

    result.type = PrimitiveType::DifferenceResult;
    result.left = std::make_shared<MeshData>(a);
    result.right = std::make_shared<MeshData>(b);

    result.C.resize(result.V.rows(), 3);

    for (int i = 0; i < result.V.rows(); ++i)
    {
        Eigen::Vector3f p = result.V.row(i).cast<float>();
        result.C.row(i) = evaluate_color_world(result, p);
    }

    result.color = 0.5 * a.color + 0.5 * b.color;

    result.V_rest = result.V;
    result.T = Eigen::Matrix4f::Identity();
}