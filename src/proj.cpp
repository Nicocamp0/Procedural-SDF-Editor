#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include <igl/read_triangle_mesh.h>
#include <igl/per_face_normals.h>

#include <imgui.h>
#include <ImGuizmo.h>
#include <igl/unproject_onto_mesh.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <limits>

#include <igl/grid.h>
#include <igl/marching_cubes.h>

int grille_taille = 30;
float object_size = 0.5f;
//couleur de base
float R = 0.8f;
float G = 0.2f;
float B = 0.2f;

// Structure pour stocker les données d'un mesh
enum PrimitiveType {
    SPHERE,
    CUBE,
    OCTAHEDRON,
    CYLINDER
};

struct MeshData {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    Eigen::MatrixXd V_rest;
    Eigen::Matrix4f T;
    Eigen::Matrix4f T_inv;
    int data_id;

    PrimitiveType type;

    float size = 0.5f;
    float radius = 0.5f;
    float height = 0.5f;

    Eigen::RowVector3d color;
};


float SDF_Sphere(Eigen::Vector3f p, float r){
    return p.norm() - r;
}

void create_grid(Eigen::MatrixXd &grid, float min, float max, int size) {
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

void create_sphere(MeshData& mesh, float r = 0.5f, int grille_taille = 30) {
    Eigen::MatrixXd grid;
    create_grid(grid, -1.5f * r, 1.5f * r, grille_taille);

    Eigen::VectorXd sdf_values(grid.rows());
    for (int i = 0; i < grid.rows(); ++i) {
        Eigen::Vector3f p = grid.row(i).cast<float>();
        sdf_values(i) = SDF_Sphere(p, r);
    }

    double iso_value = 0.0;
    igl::marching_cubes(sdf_values,grid,grille_taille, grille_taille, grille_taille,iso_value,mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();
}

float SDF_Box(Eigen::Vector3f p, Eigen::Vector3f b){
    Eigen::Vector3f q = p.cwiseAbs() - b;

    float outside = q.cwiseMax(0.0f).norm();
    float inside = std::min(std::max(q.x(), std::max(q.y(), q.z())), 0.0f);

    return outside + inside;
}

void create_cube(MeshData& mesh, float Demibox = 0.5f, int grille_taille = 30) {
    Eigen::MatrixXd grille;
    create_grid(grille, -1.5f * Demibox, 1.5f * Demibox, grille_taille);

    Eigen::VectorXd sdf_values(grille.rows());
    Eigen::Vector3f demiBox(Demibox, Demibox, Demibox);

    for (int i = 0; i < grille.rows(); i++) {
        Eigen::Vector3f point = grille.row(i).cast<float>();
        sdf_values(i) = SDF_Box(point, demiBox);
    }

    double iso_value = 0.0;
    igl::marching_cubes(sdf_values,grille,grille_taille, grille_taille, grille_taille,iso_value,mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();
}


float SDF_Octahedron(Eigen::Vector3f p, float s){
    p = p.cwiseAbs();
    float m = p.x() + p.y() + p.z() - s;

    Eigen::Vector3f q;
    if (3.0f * p.x() < m)
        q = Eigen::Vector3f(p.x(), p.y(), p.z());
    else if (3.0f * p.y() < m)
        q = Eigen::Vector3f(p.y(), p.z(), p.x());
    else if (3.0f * p.z() < m)
        q = Eigen::Vector3f(p.z(), p.x(), p.y());
    else
        return m * 0.57735027f;

    float k = std::clamp(0.5f * (q.z() - q.y() + s), 0.0f, s);
    return Eigen::Vector3f(q.x(), q.y() - s + k, q.z() - k).norm();
}

void create_octahedron(MeshData& mesh, float taille = 0.5f, int grille_taille = 30){
    Eigen::MatrixXd grille;
    create_grid(grille, -1.5f * taille, 1.5f * taille, grille_taille);

    Eigen::VectorXd sdf_values(grille.rows());
    for (int i = 0; i < grille.rows(); i++) {
        Eigen::Vector3f p = grille.row(i).cast<float>();
        sdf_values(i) = SDF_Octahedron(p, taille);
    }

    double iso_value = 0.0;
    igl::marching_cubes(sdf_values,grille,grille_taille, grille_taille, grille_taille,iso_value,mesh.V, mesh.F);

    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();
}


float SDF_Cylinder(Eigen::Vector3f p, float r, float h) {
    Eigen::Vector2f d;
    d << std::sqrt(p.x() * p.x() + p.z() * p.z()), p.y();
    d = d.cwiseAbs() - Eigen::Vector2f(r, h);

    float inside = std::min(std::max(d.x(), d.y()), 0.0f);
    float outside = d.cwiseMax(0.0f).norm();

    return inside + outside;
}

void create_cylinder(MeshData& mesh, float r = 0.5f, float h = 0.75f, int grille_taille = 30) {
    Eigen::MatrixXd grille;

    float bound = 1.5f * std::max(r, h);
    create_grid(grille, -bound, bound, grille_taille);

    Eigen::VectorXd sdf_values(grille.rows());
    for (int i = 0; i < grille.rows(); i++) {
        Eigen::Vector3f p = grille.row(i).cast<float>();
        sdf_values(i) = SDF_Cylinder(p, r, h);
    }

    double iso_value = 0.0;
    igl::marching_cubes(sdf_values,grille,grille_taille, grille_taille, grille_taille,iso_value,mesh.V, mesh.F);
    mesh.V_rest = mesh.V;
    mesh.T = Eigen::Matrix4f::Identity();
}


static Eigen::MatrixXd transform_vertices(const Eigen::MatrixXd& V, const Eigen::Matrix4f& T) {
    Eigen::MatrixXd Vt = V;
    for (int i = 0; i < V.rows(); ++i) {
        Eigen::Vector4f p;
        p << (float)V(i, 0), (float)V(i, 1), (float)V(i, 2), 1.0f;
        Eigen::Vector4f q = T * p;
        Vt(i, 0) = q(0);
        Vt(i, 1) = q(1);
        Vt(i, 2) = q(2);
    }
    return Vt;
}

static void aabb_corners(const Eigen::RowVector3d& mn, const Eigen::RowVector3d& mx, Eigen::MatrixXd& C8) {
    C8.resize(8, 3);
    C8 <<
        mn(0), mn(1), mn(2),
        mx(0), mn(1), mn(2),
        mx(0), mx(1), mn(2),
        mn(0), mx(1), mn(2),
        mn(0), mn(1), mx(2),
        mx(0), mn(1), mx(2),
        mx(0), mx(1), mx(2),
        mn(0), mx(1), mx(2);
}

static void draw_aabb(igl::opengl::glfw::Viewer& viewer, int data_id, const Eigen::MatrixXd& Vdraw) {
    Eigen::RowVector3d mn = Vdraw.colwise().minCoeff();
    Eigen::RowVector3d mx = Vdraw.colwise().maxCoeff();

    Eigen::MatrixXd C8;
    aabb_corners(mn, mx, C8);

    int E[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };

    Eigen::MatrixXd P(12, 3), Q(12, 3);
    for (int e = 0; e < 12; ++e) {
        P.row(e) = C8.row(E[e][0]);
        Q.row(e) = C8.row(E[e][1]);
    }

    viewer.data(data_id).add_edges(P, Q, Eigen::RowVector3d(1, 0, 0)); // rouge
}

int main() {
    igl::opengl::glfw::Viewer viewer;

    // Liste des meshes
    std::vector<MeshData> meshes;
    int selected_mesh = -1; // Aucun mesh sélectionné au début
    bool show_aabb = true;
    ImGuizmo::OPERATION current_gizmo_operation = ImGuizmo::TRANSLATE;
    ImGuizmo::MODE current_gizmo_mode = ImGuizmo::LOCAL;

    // ---- ImGui ----
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);

    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);

    menu.callback_draw_viewer_menu = [&]() {
        ImGui::Begin("Menu :");

        ImGui::Text("Mode :");

        if (ImGui::RadioButton("Translation", current_gizmo_operation == ImGuizmo::TRANSLATE))
            current_gizmo_operation = ImGuizmo::TRANSLATE;

        if (ImGui::RadioButton("Rotation", current_gizmo_operation == ImGuizmo::ROTATE))
            current_gizmo_operation = ImGuizmo::ROTATE;

        ImGui::Text("Repere :");

        if (ImGui::RadioButton("Local", current_gizmo_mode == ImGuizmo::LOCAL))
            current_gizmo_mode = ImGuizmo::LOCAL;

        if (ImGui::RadioButton("World", current_gizmo_mode == ImGuizmo::WORLD))
            current_gizmo_mode = ImGuizmo::WORLD;

        ImGui::Separator();

        if (ImGui::Button("Créer une sphère")) {
            MeshData new_mesh;
            create_sphere(new_mesh,object_size, grille_taille);

            int new_id = viewer.append_mesh();
            new_mesh.data_id = new_id;
            new_mesh.type=SPHERE;
            new_mesh.size=object_size;
            new_mesh.color = Eigen::RowVector3d(R, G, B);

            viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
            viewer.data(new_id).compute_normals();

            meshes.push_back(new_mesh);
            selected_mesh = (int)meshes.size() - 1;
        }

        if (ImGui::Button("Créer un cube")) {
            MeshData new_mesh;
            create_cube(new_mesh,object_size, grille_taille);
            new_mesh.color = Eigen::RowVector3d(R, G, B);

            int new_id = viewer.append_mesh();
            new_mesh.data_id = new_id;
            new_mesh.type=CUBE;
            new_mesh.size=object_size;
            new_mesh.color = Eigen::RowVector3d(R, G, B);

            viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
            viewer.data(new_id).compute_normals();

            meshes.push_back(new_mesh);
            selected_mesh = (int)meshes.size() - 1;
        }

        if(ImGui::Button("Créer un Octahedron")){
            MeshData new_mesh;
            create_octahedron(new_mesh, object_size, grille_taille);
            new_mesh.color = Eigen::RowVector3d(R, G, B);

            int new_id= viewer.append_mesh();
            new_mesh.data_id = new_id;
            new_mesh.type=OCTAHEDRON;
            new_mesh.size=object_size;
            new_mesh.color = Eigen::RowVector3d(R, G, B);

            viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
            viewer.data(new_id).compute_normals();

            meshes.push_back(new_mesh);
            selected_mesh=(int)meshes.size()-1;
        }

        if (ImGui::Button("Créer un cylindre")) {
            MeshData new_mesh;
            create_cylinder(new_mesh, object_size, object_size, grille_taille);
            new_mesh.color = Eigen::RowVector3d(R, G, B);

            int new_id = viewer.append_mesh();
            new_mesh.data_id = new_id;
            new_mesh.type=CYLINDER;
            new_mesh.size=object_size;
            new_mesh.color = Eigen::RowVector3d(R, G, B);

            viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
            viewer.data(new_id).compute_normals();

            meshes.push_back(new_mesh);
            selected_mesh = (int)meshes.size() - 1;
        }

        ImGui::Text("Taille de l'objet :");
        ImGui::Text("Parametres de creation");
        ImGui::SliderFloat("Taille objet", &object_size, 0.1f, 2.0f);
        ImGui::SliderInt("Resolution grille", &grille_taille, 5, 60);
        ImGui::Text("Couleur de creation");
        ImGui::SliderFloat("Rouge", &R, 0.0f, 1.0f);
        ImGui::SliderFloat("Vert",  &G, 0.0f, 1.0f);
        ImGui::SliderFloat("Bleu",  &B, 0.0f, 1.0f);
        ImGui::Checkbox("Show AABB", &show_aabb);


        if (!meshes.empty()) {
            ImGui::Text("Meshes existants :");
            for (size_t i = 0; i < meshes.size(); ++i) {
                ImGui::RadioButton(("Mesh " + std::to_string(i)).c_str(), &selected_mesh, (int)i);
            }
        }

        if (selected_mesh >= 0 && selected_mesh < (int)meshes.size()) {
            ImGuizmo::BeginFrame();
            ImGuizmo::SetOrthographic(false);

            auto vp = viewer.core().viewport;
            ImGuizmo::SetRect((float)vp(0), (float)vp(1), (float)vp(2), (float)vp(3));

            Eigen::Matrix4f view = viewer.core().view.cast<float>();
            Eigen::Matrix4f proj = viewer.core().proj.cast<float>();

            Eigen::Matrix4f& T = meshes[selected_mesh].T;

            ImGuizmo::Manipulate(
                view.data(),
                proj.data(),
                current_gizmo_operation,
                current_gizmo_mode,
                T.data()
            );

            if (ImGuizmo::IsUsing()) {
                meshes[selected_mesh].V = transform_vertices(meshes[selected_mesh].V_rest, T);
                viewer.data(meshes[selected_mesh].data_id).set_vertices(meshes[selected_mesh].V);
                viewer.data(meshes[selected_mesh].data_id).compute_normals();
            }
        }

        ImGui::End();
    };

    // Sélection par clic
    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer& v, int button, int modifier) -> bool {
        if (button != (int)igl::opengl::glfw::Viewer::MouseButton::Left)
            return false;

        // Si on clique sur le gizmo, on ne change pas la sélection
        if (ImGuizmo::IsOver())
            return false;

        int x = v.current_mouse_x;
        int y = v.core().viewport(3) - v.current_mouse_y;

        int best_mesh = -1;
        double best_depth = std::numeric_limits<double>::infinity();

        for (int i = 0; i < (int)meshes.size(); ++i) {
            int fid;
            Eigen::Vector3f bc;

            // On teste le mesh transformé affiché à l'écran
            const Eigen::MatrixXd& Vpick = v.data(meshes[i].data_id).V;
            const Eigen::MatrixXi& Fpick = meshes[i].F;

            bool hit = igl::unproject_onto_mesh(
                Eigen::Vector2f((float)x, (float)y),
                v.core().view,
                v.core().proj,
                v.core().viewport,
                Vpick,
                Fpick,
                fid,
                bc
            );

            if (hit) {
                Eigen::RowVector3d p =
                    bc(0) * Vpick.row(Fpick(fid, 0)) +
                    bc(1) * Vpick.row(Fpick(fid, 1)) +
                    bc(2) * Vpick.row(Fpick(fid, 2));

                Eigen::Vector4f ph;
                ph << (float)p(0), (float)p(1), (float)p(2), 1.0f;

                Eigen::Vector4f cam_p = v.core().view * ph;
                double depth = -static_cast<double>(cam_p(2));

                if (depth < best_depth) {
                    best_depth = depth;
                    best_mesh = i;
                }
            }
        }

        if (best_mesh != -1) {
            selected_mesh = best_mesh;
            return true;
        }

        return false;
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& v) {
        for (auto& mesh : meshes) {
            v.data(mesh.data_id).clear_edges();
        }

        for (int i = 0; i < (int)meshes.size(); ++i) {
            if (show_aabb) {
                draw_aabb(v, meshes[i].data_id, v.data(meshes[i].data_id).V);
            }

            // Mettre en évidence l'objet sélectionné
            if (i == selected_mesh) {
                Eigen::RowVector3d selected_color = meshes[i].color * 1.3;
                selected_color = selected_color.cwiseMin(1.0);

                v.data(meshes[i].data_id).set_face_based(true);
                v.data(meshes[i].data_id).uniform_colors(
                    selected_color,
                    Eigen::Vector3d(0.2, 0.2, 0.2),
                    Eigen::Vector3d(1.0, 1.0, 1.0)
                );
            } else {
                v.data(meshes[i].data_id).set_face_based(true);
                v.data(meshes[i].data_id).uniform_colors(
                    meshes[i].color,
                    Eigen::Vector3d(0.2, 0.2, 0.2),
                    Eigen::Vector3d(1.0, 1.0, 1.0)
                );
            }
        }
        return false;
    };

    viewer.launch();
}