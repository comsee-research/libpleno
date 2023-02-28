//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================


#include <iostream>
#include <fstream>

#include <boost/lexical_cast.hpp>
#include <boost/math/special_functions/sinc.hpp>

//#define USE_TOON
#include <libv/lma/lma.hpp>

namespace v
{
  Eigen::Matrix3d rotation_exp(const Eigen::Matrix3d &a)
  {
    double theta2 = a(0,1)*a(0,1) + a(0,2)*a(0,2) +  a(1,2)*a(1,2) + std::numeric_limits<double>::epsilon();
    double theta = std::sqrt(theta2);
    return Eigen::Matrix3d::Identity() + boost::math::sinc_pi(theta)*a+(1.0-std::cos(theta))/theta2*a*a;
  }
  
  void apply_rotation(Eigen::Matrix3d &m, const Eigen::Vector3d &d)
  {
    Eigen::Matrix3d skrew;
    skrew << 0, -d.z(), d.y(), d.z(), 0, -d.x(), -d.y(), d.x(), 0;
    m *= rotation_exp(skrew);
  }

  void apply_small_rotation_(Eigen::Matrix3d& rotation, double h, int i, int j)
  {
    const Eigen::Vector3d col_l = rotation.col(j) - rotation.col(i) * h;
    rotation.col(i) += rotation.col(j) * h;
    rotation.col(j) = col_l;
  }

  void apply_small_rotation_x(Eigen::Matrix3d& rotation, double h) { apply_small_rotation_(rotation,h,1,2); }
  void apply_small_rotation_y(Eigen::Matrix3d& rotation, double h) { apply_small_rotation_(rotation,-h,0,2); }
  void apply_small_rotation_z(Eigen::Matrix3d& rotation, double h) { apply_small_rotation_(rotation,h,0,1); }
}

// Pose camera = Rotation + Translation + intrinsics
struct Camera
{
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = {0,0,0};
  double a=0,b=0,c=0;
};

// Point 3D = Eigen::Vector3d


namespace lma
{
  // Update policy of a Camera according to 9 degres of freedom
  // The Adl parameter enable the usage of a function defined after its use
  void apply_increment(Camera& camera, const double delta[9], const Adl&)
  {
    // update rotation using exponential map : camera.rotation *= exp(skew(delta{0,1,2}))
    v::apply_rotation(camera.rotation,{delta[0],delta[1],delta[2]});

    // update translation : camera.translation += delta{3,4,5}
    camera.translation += Eigen::Map<const Eigen::Vector3d>(delta + 3);

    // update instrinsics : (camera{a,b,c} += delta{6,7,8})
    Eigen::Map<Eigen::Array3d>(&camera.a) += Eigen::Map<const Eigen::Array3d>(delta+6);
  }

  // Only for numerical derivative:
  // Update policy of a Camera according to the Ie parameter (h ~ 1e-8).
  // The Adl parameter enable the usage of a function defined after its use
  template<int I> void apply_small_increment(Camera& camera, double h, v::numeric_tag<I>, const Adl&)
  {
    if      (I == 0) v::apply_small_rotation_x(camera.rotation,h);
    else if (I == 1) v::apply_small_rotation_y(camera.rotation,h);
    else if (I == 2) v::apply_small_rotation_z(camera.rotation,h);
    else if (I == 3) camera.translation.x() += h;
    else if (I == 4) camera.translation.y() += h;
    else if (I == 5) camera.translation.z() += h;
    else if (I == 6) camera.a += h;
    else if (I == 7) camera.b += h;
    else if (I == 8) camera.c += h;
  }

  //degree of freedom of a Camera
  template<> struct Size<Camera> { enum {value = 9}; };

  // Nothing to specify for the 3D points (Eigen::Vector3d) :
  //   - degree of freedom of an Eigen::Vector is the size.
  //   - The update policy of an Eigen::Vector is operator+.
}


struct BALProblem 
{
  bool load(std::string filename)
  {
    std::ifstream file(filename);
    if (!file.is_open()) return false;

    size_t num_cameras,num_points,num_observations;

    file >> num_cameras >> num_points >> num_observations;

    std::cout << color.yellow() << 
      boost::format("Cameras[%1%], Points 3D[%2%], Observations[%3%]")%num_cameras%num_points%num_observations 
      << color.reset() << std::endl;

    point_index.resize(num_observations);
    camera_index.resize(num_observations);
    observations.resize(num_observations);
    
    cameras.resize(num_cameras);
    points3d.resize(num_points);

    for(size_t i = 0; i < observations.size(); ++i)
      file >> camera_index[i]
           >> point_index[i]
           >> observations[i].x()
           >> observations[i].y();

    for(size_t i = 0; i < num_cameras; ++i)
    {
      std::array<double,9> params;
      for(size_t k = 0 ; k < 9 ; ++k)
        file >> params[k];
      lma::apply_increment(cameras[i],params.data(),lma::Adl{});
    }

    for(size_t i = 0; i < num_points; ++i)
      file >> points3d[i].x() 
           >> points3d[i].y()
           >> points3d[i].z();

    return true;
  }

  std::vector<int> point_index;
  std::vector<int> camera_index;

  template<class T> using AlignedVector = std::vector<T,Eigen::aligned_allocator<T>>;
  AlignedVector<Eigen::Vector3d> points3d;
  AlignedVector<Eigen::Vector2d> observations;
  AlignedVector<Camera> cameras;
};


struct Reprojection
{
  const Eigen::Vector2d& obs;
  Reprojection(const Eigen::Vector2d& p2d):obs(p2d){}

  bool operator()(const Camera& camera, const Eigen::Vector3d& point, double (&error)[2]) const
  {
    const Eigen::Vector3d p = camera.rotation * point + camera.translation;
    const double
      xp = - p[0] / p[2],
      yp = - p[1] / p[2],
      r2 = xp*xp + yp*yp,
      distortion = 1.0 + r2  * (camera.b + camera.c  * r2);

    error[0] = camera.a * distortion * xp - obs[0];
    error[1] = camera.a * distortion * yp - obs[1];
    return true;
  }
};

namespace ttt
{
  template<> struct Name< Reprojection > { static std::string name(){ return "Reprojection"; } };
}

void call_lma(std::string file, double lambda, int iteration_max)
{

  BALProblem bal_problem;
  if (!bal_problem.load(file)) {
    std::cerr << "ERROR: unable to open file " << file << "\n";
    return ;
  }

  lma::Solver<Reprojection> solver(lambda,iteration_max);

  for (size_t i = 0; i < bal_problem.observations.size(); ++i)
    solver.add(
                Reprojection(bal_problem.observations[i]),
                &bal_problem.cameras[bal_problem.camera_index[i]], 
                &bal_problem.points3d[bal_problem.point_index[i]]
              );

#ifdef USE_TOON
  solver.solve(lma::TOON_DENSE_SCHUR,lma::enable_verbose_output());
#else
  solver.solve(lma::DENSE_SCHUR,lma::enable_verbose_output());
#endif
}



int main(int argc, char** argv)
{
  if (argc == 4) 
    call_lma(argv[1],boost::lexical_cast<double>(argv[2]),boost::lexical_cast<int>(argv[3]));
  else
    std::cerr << "usage: test-lma3d <bal_problem> initial_lambda nb_iteration_max\n";

  return 0;
}

