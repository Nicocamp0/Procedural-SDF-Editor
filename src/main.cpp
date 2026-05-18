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
#include <limits>
#include <algorithm>

#include <igl/grid.h>
#include <igl/marching_cubes.h>


// Include maison mais ne sais pas encore s'ils sont tous utiles => faudra voir une fois nettoyage terminé
#include "../include/Creation_Primitive.h"
#include "../include/Mesh.h"
#include "../include/SDF.h"

// Pour utilisation constante PI et fonction sinus/cosinus !!! 
#define _USE_MATH_DEFINES
#include <math.h>




/// 
/// ENCORE A NETTOYER ET A CORRIGER ICI MAIS ON AVANCE BIEN :)
///

int grille_taille = 30;
float object_size = 0.5f;
float primitive_color[3] = {1.0f, 0.5f, 0.2f};

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

static void apply_color(MeshData& mesh, float color[3])
{
    mesh.color = Eigen::RowVector3d(color[0], color[1], color[2]);
    mesh.C = mesh.color.replicate(mesh.V.rows(), 1);
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
    if (Vdraw.rows() == 0) {
        return;
    }

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

    viewer.data(data_id).add_edges(P, Q, Eigen::RowVector3d(1, 0, 0));
}

int main() {
    igl::opengl::glfw::Viewer viewer;

    // Liste des meshes
    std::vector<MeshData> meshes;
    int selected_mesh = -1; // Aucun mesh sélectionné au début
    bool show_aabb = true;

    // Multi-sélection pour l'union
    std::vector<char> union_selected;
    std::vector<char> intersection_selected;
    std::vector<char> difference_selected;

    // Gizmo
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

             
        // Section sur les primitives géométriques
        if(ImGui::CollapsingHeader("Primitives")){

            if (ImGui::Button("Créer une sphère")) {
                MeshData new_mesh;
                create_sphere(new_mesh, object_size, grille_taille);
                apply_color(new_mesh, primitive_color);

                int new_id = viewer.append_mesh();
                new_mesh.data_id = new_id;

                viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
                viewer.data(new_id).set_colors(new_mesh.C);
                viewer.data(new_id).compute_normals();

                meshes.push_back(new_mesh);
                union_selected.push_back(0);
                intersection_selected.push_back(0);
                difference_selected.push_back(0);
                selected_mesh = (int)meshes.size() - 1;
            }

            if (ImGui::Button("Créer un cube")) {
                MeshData new_mesh;
                create_cube(new_mesh, object_size, grille_taille);
                apply_color(new_mesh, primitive_color);

                int new_id = viewer.append_mesh();
                new_mesh.data_id = new_id;

                viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
                viewer.data(new_id).set_colors(new_mesh.C);
                viewer.data(new_id).compute_normals();

                meshes.push_back(new_mesh);
                union_selected.push_back(0);
                intersection_selected.push_back(0);
                difference_selected.push_back(0);
                selected_mesh = (int)meshes.size() - 1;
            }

            if (ImGui::Button("Créer un Octahedron")) {
                MeshData new_mesh;
                create_octahedron(new_mesh, object_size, grille_taille);
                apply_color(new_mesh, primitive_color);

                int new_id = viewer.append_mesh();
                new_mesh.data_id = new_id;

                viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
                viewer.data(new_id).set_colors(new_mesh.C);
                viewer.data(new_id).compute_normals();

                meshes.push_back(new_mesh);
                union_selected.push_back(0);
                intersection_selected.push_back(0);
                difference_selected.push_back(0);
                selected_mesh = (int)meshes.size() - 1;
            }

            if (ImGui::Button("Créer un cylindre")) {
                MeshData new_mesh;
                create_cylinder(new_mesh, object_size, object_size, grille_taille);
                apply_color(new_mesh, primitive_color);

                int new_id = viewer.append_mesh();
                new_mesh.data_id = new_id;

                viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
                viewer.data(new_id).set_colors(new_mesh.C);
                viewer.data(new_id).compute_normals();

                meshes.push_back(new_mesh);
                union_selected.push_back(0);
                intersection_selected.push_back(0);
                difference_selected.push_back(0);
                selected_mesh = (int)meshes.size() - 1;
            }

            if (ImGui::Button("Créer un torus")) {
                MeshData new_mesh;
                create_torus(new_mesh, object_size * 0.3f, object_size, grille_taille);
                apply_color(new_mesh, primitive_color);

                int new_id = viewer.append_mesh();
                new_mesh.data_id = new_id;

                viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
                viewer.data(new_id).set_colors(new_mesh.C);
                viewer.data(new_id).compute_normals();

                meshes.push_back(new_mesh);
                union_selected.push_back(0);
                intersection_selected.push_back(0);
                difference_selected.push_back(0);
                selected_mesh = (int)meshes.size() - 1;
            }

            if (ImGui::Button("Créer un link")) {
                MeshData new_mesh;
                create_link(new_mesh, object_size, object_size * 0.25f, object_size, grille_taille);
                apply_color(new_mesh, primitive_color);

                int new_id = viewer.append_mesh();
                new_mesh.data_id = new_id;

                viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
                viewer.data(new_id).set_colors(new_mesh.C);
                viewer.data(new_id).compute_normals();

                meshes.push_back(new_mesh);
                union_selected.push_back(0);
                intersection_selected.push_back(0);
                difference_selected.push_back(0);
                selected_mesh = (int)meshes.size() - 1;
            }

            if (ImGui::Button("Créer un cône")) {
                MeshData new_mesh;
                create_cone(new_mesh, object_size, M_PI / 4.0f, grille_taille);
                apply_color(new_mesh, primitive_color);

                int new_id = viewer.append_mesh();
                new_mesh.data_id = new_id;

                viewer.data(new_id).set_mesh(new_mesh.V, new_mesh.F);
                viewer.data(new_id).set_colors(new_mesh.C);
                viewer.data(new_id).compute_normals();

                meshes.push_back(new_mesh);
                union_selected.push_back(0);
                intersection_selected.push_back(0);
                difference_selected.push_back(0);
                selected_mesh = (int)meshes.size() - 1;
            }
        }


        
        
        // Section sur les opérations qu'on peut appliquer sur primitives
        if(ImGui::CollapsingHeader("Opérations")){

            // Bouton union des objets sélectionnés
            if (ImGui::Button("Union des objets sélectionnés")) {
                std::vector<int> selected_ids;

                for (int i = 0; i < (int)union_selected.size(); ++i) {
                    if (union_selected[i]) {
                        selected_ids.push_back(i);
                    }
                }

                if (selected_ids.size() >= 2) {
                    MeshData result;
                    create_union_mesh(meshes[selected_ids[0]], meshes[selected_ids[1]], result);

                    for (size_t k = 2; k < selected_ids.size(); ++k) {
                        MeshData temp;
                        create_union_mesh(result, meshes[selected_ids[k]], temp);
                        result = temp;
                    }

                    int new_id = viewer.append_mesh();
                    result.data_id = new_id;

                    viewer.data(new_id).set_mesh(result.V, result.F);
                    viewer.data(new_id).compute_normals();

                    for (int idx : selected_ids) {
                        viewer.data(meshes[idx].data_id).clear();
                    }

                    std::sort(selected_ids.rbegin(), selected_ids.rend());
                    for (int idx : selected_ids) {
                        meshes.erase(meshes.begin() + idx);
                        union_selected.erase(union_selected.begin() + idx);
                        intersection_selected.erase(intersection_selected.begin() + idx);
                        difference_selected.erase(difference_selected.begin() + idx);
                    }

                    meshes.push_back(result);
                    union_selected.push_back(0);
                    intersection_selected.push_back(0);
                    difference_selected.push_back(0);
                    selected_mesh = (int)meshes.size() - 1;
                }
            }

            if (ImGui::Button("Intersection des objets sélectionnés")) {
                std::vector<int> selected_ids;

                for (int i = 0; i < (int)intersection_selected.size(); ++i) {
                    if (intersection_selected[i]) {
                        selected_ids.push_back(i);
                    }
                }

                if (selected_ids.size() >= 2) {
                    MeshData result;
                    create_intersection_mesh(meshes[selected_ids[0]], meshes[selected_ids[1]], result);

                    for (size_t k = 2; k < selected_ids.size(); ++k) {
                        MeshData temp;
                        create_intersection_mesh(result, meshes[selected_ids[k]], temp);
                        result = temp;
                    }

                    int new_id = viewer.append_mesh();
                    result.data_id = new_id;

                    viewer.data(new_id).set_mesh(result.V, result.F);
                    viewer.data(new_id).compute_normals();

                    for (int idx : selected_ids) {
                        viewer.data(meshes[idx].data_id).clear();
                    }

                    std::sort(selected_ids.rbegin(), selected_ids.rend());
                    for (int idx : selected_ids) {
                        meshes.erase(meshes.begin() + idx);
                        union_selected.erase(union_selected.begin() + idx);
                        intersection_selected.erase(intersection_selected.begin() + idx);
                        difference_selected.erase(difference_selected.begin() + idx);
                    }

                    meshes.push_back(result);
                    union_selected.push_back(0);
                    intersection_selected.push_back(0);
                    difference_selected.push_back(0);
                    selected_mesh = (int)meshes.size() - 1;
                }
            }

            if (ImGui::Button("Différence des objets sélectionnés")) {
                std::vector<int> selected_ids;

                for (int i = 0; i < (int)difference_selected.size(); ++i) {
                    if (difference_selected[i]) {
                        selected_ids.push_back(i);
                    }
                }

                if (selected_ids.size() >= 2) {
                    MeshData result;
                    create_difference_mesh(meshes[selected_ids[0]], meshes[selected_ids[1]], result);

                    for (size_t k = 2; k < selected_ids.size(); ++k) {
                        MeshData temp;
                        create_difference_mesh(result, meshes[selected_ids[k]], temp);
                        result = temp;
                    }

                    int new_id = viewer.append_mesh();
                    result.data_id = new_id;

                    viewer.data(new_id).set_mesh(result.V, result.F);
                    viewer.data(new_id).compute_normals();

                    for (int idx : selected_ids) {
                        viewer.data(meshes[idx].data_id).clear();
                    }

                    std::sort(selected_ids.rbegin(), selected_ids.rend());
                    for (int idx : selected_ids) {
                        meshes.erase(meshes.begin() + idx);
                        union_selected.erase(union_selected.begin() + idx);
                        intersection_selected.erase(intersection_selected.begin() + idx);
                        difference_selected.erase(difference_selected.begin() + idx);
                    }

                    meshes.push_back(result);
                    union_selected.push_back(0);
                    intersection_selected.push_back(0);
                    difference_selected.push_back(0);
                    selected_mesh = (int)meshes.size() - 1;
                }
            }
        }

        ImGui::Separator();

         // Paramètres dde création dans l'UI
        if(ImGui::CollapsingHeader("Parametres de creation")){
        ImGui::SliderFloat("Taille objet", &object_size, 0.1f, 2.0f);
        ImGui::SliderInt("Resolution grille", &grille_taille, 5, 60);
        ImGui::SliderFloat("R", &primitive_color[0], 0.0f, 1.0f);
        ImGui::SliderFloat("G", &primitive_color[1], 0.0f, 1.0f);
        ImGui::SliderFloat("B", &primitive_color[2], 0.0f, 1.0f);
        }

        ImGui::Checkbox("Show AABB", &show_aabb);

        ImGui::Separator();
        ImGui::Text("Gizmo");

        if (!meshes.empty()) {
            ImGui::Text("Meshes existants :");

            for (size_t i = 0; i < meshes.size(); ++i) {
                std::string label_select = "Mesh " + std::to_string(i);
                ImGui::RadioButton(label_select.c_str(), &selected_mesh, (int)i);

                ImGui::SameLine();

                std::string label_union = "Union##" + std::to_string(i);
                bool checked = (union_selected[i] != 0);
                if (ImGui::Checkbox(label_union.c_str(), &checked)) {
                    union_selected[i] = checked ? 1 : 0;
                }

                ImGui::SameLine();

                std::string label_intersection = "Inter##" + std::to_string(i);
                bool checked_intersection = (intersection_selected[i] != 0);
                if (ImGui::Checkbox(label_intersection.c_str(), &checked_intersection)) {
                    intersection_selected[i] = checked_intersection ? 1 : 0;
                }

                ImGui::SameLine();

                std::string label_difference = "Diff##" + std::to_string(i);
                bool checked_difference = (difference_selected[i] != 0);
                if (ImGui::Checkbox(label_difference.c_str(), &checked_difference)) {
                    difference_selected[i] = checked_difference ? 1 : 0;
                }
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
            v.data(meshes[i].data_id).set_face_based(false);

            if (meshes[i].C.rows() == meshes[i].V.rows()) {
                v.data(meshes[i].data_id).set_colors(meshes[i].C);
            }
        }
        return false;
    };

    viewer.launch();
}