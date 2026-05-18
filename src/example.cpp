#include <igl/opengl/glfw/Viewer.h>
#include <igl/read_triangle_mesh.h>

#include <igl/adjacency_matrix.h>
#include <igl/triangle_triangle_adjacency.h>
#include <igl/vertex_triangle_adjacency.h>

#include <Eigen/Sparse>
#include <iostream>
#include <vector>

#include <igl/per_vertex_normals.h>
#include <igl/per_face_normals.h>
//#include <igl/normalize_row_lengths.h>


int main(int argc, char *argv[])
{
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  if(!igl::read_triangle_mesh("/adhome/v/ve/ventadoux/Bureau/Geodigi/libigl-example-project/data/bunny.off", V, F))
  {
    std::cerr << "Erreur: impossible de charger le maillage\n";
    return 1;
  }

  std::cout << "#V = " << V.rows() << ", #F = " << F.rows() << std::endl;

  Eigen::SparseMatrix<int> A;
  igl::adjacency_matrix(F, A);

  int i = 0, j = 10;
  std::cout << "A("<<i<<","<<j<<")=" << A.coeff(i,j) << std::endl;

  Eigen::MatrixXi TT, TTi;
  igl::triangle_triangle_adjacency(F, TT, TTi);

  /*for(int f=0; f<F.rows(); ++f){
    std::cout << "Face " << f << " voisins : "
              << TT(f,0) << " " << TT(f,1) << " " << TT(f,2) << "\n";
  }*/

  int f0 = 0;
  std::cout << "Face " << f0 << " -> sommets : "
            << F(f0,0) << " " << F(f0,1) << " " << F(f0,2) << "\n";

  /*for(int k=0;k<3;k++){
    int vi = F(f0,k);
    std::cout << "v" << vi << " = " << V.row(vi) << "\n";
  }*/

  std::vector<std::vector<int>> VF;
  std::vector<std::vector<int>> VFi;

  igl::vertex_triangle_adjacency(V, F, VF, VFi);

 /* for(int v=0; v<(int)VF.size(); ++v){
    std::cout << "Sommet " << v << " faces : ";
    for(int ff : VF[v]) std::cout << ff << " ";
    std::cout << "\n";
  }*/


  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  viewer.data().set_face_based(true);
  viewer.launch();
}
