#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

#include <igl/read_triangle_mesh.h>
#include <igl/per_face_normals.h>

#include <imgui.h>
#include <ImGuizmo.h>

#include <Eigen/Core>
#include <iostream>
#include <vector>


static Eigen::MatrixXd transform_vertices(const Eigen::MatrixXd& V, const Eigen::Matrix4f& T)
{
  Eigen::MatrixXd Vt = V;
  for(int i=0;i<V.rows();++i){
    Eigen::Vector4f p;
    p << (float)V(i,0), (float)V(i,1), (float)V(i,2), 1.0f;
    Eigen::Vector4f q = T * p;
    Vt(i,0)=q(0); Vt(i,1)=q(1); Vt(i,2)=q(2);
  }
  return Vt;
}

static void aabb_corners(const Eigen::RowVector3d& mn, const Eigen::RowVector3d& mx, Eigen::MatrixXd& C8)
{
  C8.resize(8,3);
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

static void draw_aabb(igl::opengl::glfw::Viewer& viewer, int data_id, const Eigen::MatrixXd& Vdraw)
{
  Eigen::RowVector3d mn = Vdraw.colwise().minCoeff();
  Eigen::RowVector3d mx = Vdraw.colwise().maxCoeff();

  Eigen::MatrixXd C8;
  aabb_corners(mn, mx, C8);

  int E[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
  };

  Eigen::MatrixXd P(12,3), Q(12,3);
  for(int e=0;e<12;++e){
    P.row(e) = C8.row(E[e][0]);
    Q.row(e) = C8.row(E[e][1]);
  }

  viewer.data(data_id).add_edges(P, Q, Eigen::RowVector3d(1,0,0)); // rouge
}

int main()
{
  igl::opengl::glfw::Viewer viewer;

  // 2 vues
  viewer.core_list.resize(2);
  viewer.core_list[1] = viewer.core_list[0];
  int win_w = 1500;
  int win_h = 1600;


  auto update_viewports = [&]()
  {
    float w = (float)win_w;
    float h = (float)win_h;

    // gauche
    viewer.core_list[0].viewport = Eigen::Vector4f(0, 0, w/2.0f, h);
    // droite
    viewer.core_list[1].viewport = Eigen::Vector4f(w/2.0f, 0, w/2.0f, h);
  };


  viewer.callback_post_resize = [&](igl::opengl::glfw::Viewer&, int w, int h)
  {
    win_w = w;
    win_h = h;
    update_viewports();
    return false;
  };


  // ---- Mesh 0 & 1 ----
  Eigen::MatrixXd V0, V1, V0_rest, V1_rest;
  Eigen::MatrixXi F0, F1;

  if(!igl::read_triangle_mesh("../data/bunny.off", V0_rest, F0)) return 1;
  if(!igl::read_triangle_mesh("../data/cube.off",  V1_rest, F1)) return 1;

  V0 = V0_rest; V1 = V1_rest;

  int id0 = viewer.data().id;
  int id1 = viewer.append_mesh();

  viewer.data(id0).set_mesh(V0, F0);
  viewer.data(id1).set_mesh(V1, F1);
  for (int cid = 0; cid < (int)viewer.core_list.size(); ++cid)
  {
    viewer.data(id0).set_visible(true, cid);
    viewer.data(id1).set_visible(true, cid);
  }


  // Transforms
  Eigen::Matrix4f T0 = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f T1 = Eigen::Matrix4f::Identity();
  T1(0,3) = 1.0f; // décale cube en x au départ

  // Applique transforms au départ
  viewer.data(id0).set_vertices(transform_vertices(V0_rest, T0));
  viewer.data(id1).set_vertices(transform_vertices(V1_rest, T1));
  viewer.data(id0).compute_normals();
  viewer.data(id1).compute_normals();

  // ---- ImGui ----
  igl::opengl::glfw::imgui::ImGuiPlugin plugin;
  viewer.plugins.push_back(&plugin);

  igl::opengl::glfw::imgui::ImGuiMenu menu;
  plugin.widgets.push_back(&menu);

  int selected = 0; // 0 ou 1
  bool show_aabb = true;

  menu.callback_draw_viewer_menu = [&]()
  {
    ImGui::Begin("Exo4");

    ImGui::Text("Selection mesh");
    ImGui::RadioButton("Mesh 0 (bunny)", &selected, 0);
    ImGui::RadioButton("Mesh 1 (cube)", &selected, 1);

    ImGui::Checkbox("Show AABB", &show_aabb);

    ImGui::Separator();
    ImGui::Text("Gizmo (translate)");

    // Prépare matrices cam pour ImGuizmo (col-major float[16])
    ImGuizmo::BeginFrame();
    ImGuizmo::SetOrthographic(false);

    // viewport de la vue 0 (gizmo dans la vue gauche)
    auto vp = viewer.core(0).viewport;
    ImGuizmo::SetRect(vp(0), vp(1), vp(2), vp(3));

    // Matrices view/proj depuis libigl
    Eigen::Matrix4f view = viewer.core(0).view.cast<float>();
    Eigen::Matrix4f proj = viewer.core(0).proj.cast<float>();

    Eigen::Matrix4f& T = (selected==0) ? T0 : T1;

    // gizmo translation
    ImGuizmo::Manipulate(view.data(), proj.data(),
                         ImGuizmo::TRANSLATE, ImGuizmo::LOCAL,
                         T.data());

    // si modif -> update vertices
    if(ImGuizmo::IsUsing())
    {
      if(selected==0){
        viewer.data(id0).set_vertices(transform_vertices(V0_rest, T0));
        viewer.data(id0).compute_normals();
      }else{
        viewer.data(id1).set_vertices(transform_vertices(V1_rest, T1));
        viewer.data(id1).compute_normals();
      }
    }

    ImGui::End();
  };

  // Dessin AABB à chaque frame (simple)
  viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& v)
  {
    update_viewports();
    if(!show_aabb) return false;

    // Nettoie les edges précédents (sinon ça s’accumule)
    v.data(id0).clear_edges();
    v.data(id1).clear_edges();

    draw_aabb(v, id0, v.data(id0).V);
    draw_aabb(v, id1, v.data(id1).V);
    return false;
  };

  viewer.callback_post_draw = [&](igl::opengl::glfw::Viewer& v)
  {
    // Copie caméra gauche -> droite APRÈS les updates d'input
    v.core_list[1].view = v.core_list[0].view;
    v.core_list[1].proj = v.core_list[0].proj;

    v.core_list[1].camera_eye    = v.core_list[0].camera_eye;
    v.core_list[1].camera_up     = v.core_list[0].camera_up;
    v.core_list[1].camera_center = v.core_list[0].camera_center;
    v.core_list[1].camera_zoom   = v.core_list[0].camera_zoom;

    return false;
  };


  update_viewports();
  viewer.launch();
}

