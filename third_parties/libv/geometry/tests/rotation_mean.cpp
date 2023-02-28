/**

\file
\author Simeon Schwab
\copyright 2014 Institut Pascal
\privatesection

*/

#include <cmath>
#include <vector>
#include <libv/core/test.hpp>
#include <libv/geometry/rotation.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>

#include <fenv.h>

using namespace v;

typedef std::vector<Eigen::Matrix3d,Eigen::aligned_allocator<Eigen::Matrix3d> > RotationsVector;

double test_2_rotations()
{
  const size_t n=100;
  Eigen::Matrix3Xd angle_axis =  (0.4*M_PI/std::sqrt(3.))*Eigen::Matrix3Xd::Random(3,n);

  RotationsVector rotations(n);

  for(size_t i=0; i<n; ++i)
  {
    rotations[i] = Eigen::AngleAxisd(angle_axis.col(i).norm(), angle_axis.col(i).normalized());
  }

  double err = -1.;
  for (size_t i=0; i<n-1; ++i)
  {

    Eigen::Matrix3d rm = v::geometry::rotation_mean(rotations.data()+i, rotations.data()+i+2);
    Eigen::Matrix3d ri = v::geometry::rotation_linear_interpolation(rotations[i],rotations[i+1], 0.5);

    double e = std::max(v::geometry::geodesic_distance(rm, ri), err);

    if (std::isfinite(e))
      err = std::max(err,e);
  }

  std::cout << "2 rot. max. err=" << err << std::endl;

  return err;
}

double test_orthogonalize()
{
  const size_t n=100;
  double err = -1.;

  for (size_t i=0; i<n; ++i)
  {
    Eigen::Matrix3d m = Eigen::Matrix3d::Random(3,3);
    Eigen::Matrix3d r = v::rotation_orthogonalize(m);

    double err_ortho = std::abs(r.determinant()-1.);
    double err_norm = std::abs((r*r.transpose()-Eigen::Matrix3d::Identity()).norm());

    err = std::max(err, std::max(err_ortho, err_norm));
  }

  std::cout << "orthogonalize max. err=" << err << std::endl;

  return err;
}

double test_stability()
{
  const size_t n=100;
  Eigen::Matrix3Xd angle_axis =  (0.45*M_PI/std::sqrt(3.))*Eigen::Matrix3Xd::Random(3,n);

  RotationsVector rotations(n);

  for(size_t i=0; i<n; ++i)
  {
    rotations[i] = Eigen::AngleAxisd(angle_axis.col(i).norm(), angle_axis.col(i).normalized());
  }

  double err = -1.;
  Eigen::Matrix3d mean = v::geometry::rotation_mean(rotations.begin(), rotations.end());
  for (size_t i=0; i<100; ++i)
  {
    std::random_shuffle(rotations.begin(), rotations.end());
    Eigen::Matrix3d m = v::geometry::rotation_mean(rotations.begin(), rotations.end());

    double e = v::geometry::geodesic_distance(m,mean);

    if (std::isfinite(e))
      err = std::max(err,e);
  }

  std::cout << "stability max. err=" << err << std::endl;

  return err;
}

int main(int, char **)
{
  V_TEST_LT(test_2_rotations(), 1e-7);
  V_TEST_LT(test_stability(), 1e-7);
  V_TEST_LT(test_orthogonalize(),1e-10);
}

