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
#include <igl/opengl/glfw/Viewer.h>
#include <igl/get_seconds.h>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>


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
bool readOBJFile(const std::string &filename, Eigen::MatrixXd &V, Eigen::MatrixXd &N, Eigen::MatrixXi &F) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error: Cannot open OBJ file: " << filename << std::endl;
        return false;
    }

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3i> faces;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string type;
        iss >> type;

        if (type == "v") {
            double x, y, z;
            if (!(iss >> x >> y >> z)) continue;
            vertices.emplace_back(x, y, z);
        } else if (type == "vn") {
            double nx, ny, nz;
            if (!(iss >> nx >> ny >> nz)) continue;
            normals.emplace_back(nx, ny, nz);
        } else if (type == "f") {
            std::vector<int> face;
            std::string vertex;
            while (iss >> vertex) {
                std::istringstream viss(vertex);
                std::string index;
                std::getline(viss, index, '/');
                try {
                    face.push_back(std::stoi(index) - 1);
                } catch (...) {
                    std::cerr << "Warning: Skipping invalid face line: " << line << std::endl;
                    face.clear();
                    break;
                }
            }
            if (face.size() == 3)
                faces.emplace_back(face[0], face[1], face[2]);
        }
    }
    file.close();

    V.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) V.row(i) = vertices[i];

    N.resize(normals.size(), 3);
    for (size_t i = 0; i < normals.size(); ++i) N.row(i) = normals[i];

    F.resize(faces.size(), 3);
    for (size_t i = 0; i < faces.size(); ++i) F.row(i) = faces[i];

    std::cout << "Loaded " << V.rows() << " vertices, " << N.rows() << " normals, and " << F.rows() << " faces from OBJ file." << std::endl;
    return true;
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


    Eigen::MatrixXd P;
    Eigen::MatrixXd N;
    Eigen::MatrixXi F;

    if (!readOBJFile(argv[1], P, N, F)) {
        std::cerr << "Error: Failed to read OBJ file: " << argv[1] << std::endl;
        return EXIT_FAILURE;
    }

  std::cout << "Loaded " << P.rows() << " vertices and " << N.rows() << " normals from OBJ file." << std::endl;

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
  std::cout << "done " << std::endl;


  // Generate a list of random query points in the bounding box
  Eigen::MatrixXd Q = Eigen::MatrixXd::Random(1000000,3);
  const Eigen::RowVector3d Vmin = P.colwise().minCoeff();
  const Eigen::RowVector3d Vmax = P.colwise().maxCoeff();
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


  writeToOBJ(argv[2] , QiP);
  std::cout << "writing " << QiP.rows() << " vertices to a OBJ file." << std::endl;


  // Visualization
  igl::opengl::glfw::Viewer viewer;
  // For dislpaying normals as little line segments
  Eigen::MatrixXd PN(2*P.rows(),3);
  Eigen::MatrixXi E(P.rows(),2);
  const double bbd = igl::bounding_box_diagonal(P);
  for(int p = 0;p<P.rows();p++)
  {
    E(p,0) = 2*p;
    E(p,1) = 2*p+1;
    PN.row(E(p,0)) = P.row(p);
    PN.row(E(p,1)) = P.row(p)+bbd*0.01*N.row(p);
  }

  bool show_P = false;
  int show_Q = 0;

  int query_data = 0;
//  viewer.data_list[query_data].set_mesh(V,F);
  viewer.data_list[query_data].clear();
  viewer.data_list[query_data].point_size = 2;
  viewer.append_mesh();
  int object_data = 1;
//  viewer.data_list[object_data].set_mesh(V,F);
  viewer.data_list[object_data].point_size = 5;

  const auto update = [&]()
  {
    viewer.data_list[query_data].clear();
    switch(show_Q)
    {
      case 1:
        // show all Q
        viewer.data_list[query_data].set_points(Q,Eigen::RowVector3d(0.996078,0.760784,0.760784));
        break;
      case 2:
        // show all Q inside
        if(show_P)
        {
          viewer.data_list[query_data].set_points(QiP,Eigen::RowVector3d(0.564706,0.847059,0.768627));
        }
        break;
    }
    
    viewer.data_list[object_data].clear();
    if(show_P)
    {
      viewer.data_list[object_data].set_points(P,Eigen::RowVector3d(1,1,1));
      viewer.data_list[object_data].set_edges(PN,E,Eigen::RowVector3d(0.8,0.8,0.8));
    }
  };



  viewer.callback_key_pressed = 
    [&](igl::opengl::glfw::Viewer &, unsigned int key, int mod)
  {
    switch(key)
    {
      default: 
        return false;
      case '1':
        show_P = !show_P;
        break;
      case '2':
        show_Q = (show_Q+1) % 3;
        break;
    }
    update();
    return true;
  };

  std::cout<<R"(
FastWindingNumber
  1  Toggle point cloud and triangle soup
  2  Toggle hiding query points, showing query points, showing inside queries
)";

  update();
  viewer.launch();

}
