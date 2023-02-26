#include <igl/fast_winding_number.h>
#include <igl/read_triangle_mesh.h>
#include <igl/slice_mask.h>
#include <Eigen/Geometry>
#include <igl/octree.h>
#include <igl/barycenter.h>
#include <igl/knn.h>
#include <igl/random_points_on_mesh.h>
#include <igl/bounding_box_diagonal.h>
#include <igl/per_face_normals.h>
#include <igl/copyleft/cgal/point_areas.h>
#include <igl/get_seconds.h>
#include <igl/writeOBJ.h>
#include <iostream>
#include <fstream>
#include <cstdlib>

void writeToOBJ( const std::string str, const Eigen::MatrixXd& V)
{
//Frank Dou: write points to a .obj file.

  FILE * obj_file = fopen(str.c_str(),"w");
  for(int i = 0;i<V.rows();i++)
  {
    fprintf(obj_file,"v");
    for(int j = 0;j<V.cols();++j)
    {
      fprintf(obj_file," %0.17g", V(i,j));
    }
    fprintf(obj_file,"\n");
  }
   fclose(obj_file);
}



int main(int argc, char *argv[])
{
  const auto time = [](std::function<void(void)> func)->double
  {
    const double t_before = igl::get_seconds();
    func();
    const double t_after = igl::get_seconds();
    return t_after-t_before;
  };

  Eigen::MatrixXd V;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(argc>1?argv[1]:TUTORIAL_SHARED_PATH "/bunny.off",V,F);

  // Sample mesh for point cloud

  Eigen::MatrixXd P,N;
  {
    Eigen::VectorXi I;
    Eigen::SparseMatrix<double> B;
    igl::random_points_on_mesh(10000,V,F,B,I);
    P = B*V;
    Eigen::MatrixXd FN;
    igl::per_face_normals(V,F,FN);
    N.resize(P.rows(),3);
    for(int p = 0;p<I.rows();p++)
    {
      N.row(p) = FN.row(I(p));
    }
  }
  // Build octree
  std::vector<std::vector<int > > O_PI;
  Eigen::MatrixXi O_CH;
  Eigen::MatrixXd O_CN;
  Eigen::VectorXd O_W;
  igl::octree(P,O_PI,O_CH,O_CN,O_W);
  Eigen::VectorXd A;
  {
    Eigen::MatrixXi I;
    igl::knn(P,20,O_PI,O_CH,O_CN,O_W,I);
    // CGAL is only used to help get point areas
    igl::copyleft::cgal::point_areas(P,I,N,A);
  }
  if(argc<=1)
  {
    // corrupt mesh
    Eigen::MatrixXd BC;
    igl::barycenter(V,F,BC);
    Eigen::MatrixXd OV = V;
    V.resize(F.rows()*3,3);
    for(int f = 0;f<F.rows();f++)
    {
      for(int c = 0;c<3;c++)
      {
        int v = f+c*F.rows();
        // random rotation about barycenter
        Eigen::AngleAxisd R(
          0.5*static_cast <double> (rand()) / static_cast <double> (RAND_MAX),
          Eigen::Vector3d::Random(3,1));
        V.row(v) = (OV.row(F(f,c))-BC.row(f))*R.matrix()+BC.row(f);
        F(f,c) = v;
      }
    }
  }
  // Generate a list of random query points in the bounding box
  Eigen::MatrixXd Q = Eigen::MatrixXd::Random(1000000,3);
  const Eigen::RowVector3d Vmin = V.colwise().minCoeff();
  const Eigen::RowVector3d Vmax = V.colwise().maxCoeff();
  const Eigen::RowVector3d Vdiag = Vmax-Vmin;
  for(int q = 0;q<Q.rows();q++)
  {
    Q.row(q) = (Q.row(q).array()*0.5+0.5)*Vdiag.array() + Vmin.array();
  }

  // Positions of points inside of point cloud P
  Eigen::MatrixXd QiP;
  {
    Eigen::MatrixXd O_CM;
    Eigen::VectorXd O_R;
    Eigen::MatrixXd O_EC;
    printf("     point cloud precomputation (% 8ld points):    %g secs\n",
      P.rows(),
      time([&](){igl::fast_winding_number(P,N,A,O_PI,O_CH,2,O_CM,O_R,O_EC);}));
    Eigen::VectorXd WiP;
    printf("        point cloud evaluation  (% 8ld queries):   %g secs\n",
      Q.rows(),
      time([&](){igl::fast_winding_number(P,N,A,O_PI,O_CH,O_CM,O_R,O_EC,Q,2,WiP);}));
    igl::slice_mask(Q,(WiP.array()>0.5).eval(),1,QiP);
  }

   writeToOBJ(argc>1?argv[2]:TUTORIAL_SHARED_PATH "./pc.obj", P);
   writeToOBJ(argc>1?argv[3]:TUTORIAL_SHARED_PATH "./inner_points.obj", QiP);


}
