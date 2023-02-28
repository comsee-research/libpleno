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
#include <unsupported/Eigen/MatrixFunctions>

using namespace v;

double geodesic_distance_with_eigen(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
{
  return (a.transpose() * b).log().norm();
}

Eigen::Matrix3d rotation_linear_interpolation_with_log_eigen
( const Eigen::Matrix3d &a
, const Eigen::Matrix3d &b
, double t)
{
  return a * (t * (a.transpose() * b).log()).exp();
}

double TestGeodesicDistance(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
  {
    return std::fabs(geodesic_distance(a, b)-geodesic_distance_with_eigen(a, b));
  }

double TestExp(const Eigen::Matrix3d &a, const Eigen::Matrix3d &)
  {
    Eigen::Matrix3d tmpl = rotation_log(a);
    Eigen::Matrix3d tmp = rotation_exp(tmpl);
    return (a-tmp).norm();
  }

double TestLog(const Eigen::Matrix3d &a, const Eigen::Matrix3d &)
  {
    Eigen::Matrix3d tmp = rotation_log(a);
    return (a.log()-tmp).norm();
  }

double TestLinearInterp(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
  {
    double t = double(std::rand())/double(RAND_MAX);
    return (rotation_linear_interpolation_with_log_eigen(a,b,t)-rotation_linear_interpolation(a,b,t)).norm();
  }

double TestApplyRotation(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b)
  {
    Eigen::Matrix3d aa = a;
    Eigen::Matrix3d lb = rotation_log(b);
    apply_rotation(aa, Eigen::Vector3d(lb(2,1),lb(0,2),lb(1,0)));
    return (aa-a*b).norm();
  }

template <class T>
double test(T t)
{
  const size_t n=10000;
  Eigen::VectorXd theta;
  theta.setLinSpaced(n,-2*3.14,2*3.14);

  // test unstable values:
  theta(0) = 0-std::numeric_limits<double>::epsilon();
  theta(1) = 0;
  theta(2) = 0+std::numeric_limits<double>::epsilon();
  // with pi the exp and log map are undefined, and it is unstable near from pi
  theta(3) = M_PI-1e-4;
  theta(4) = M_PI+1e-4;

  Eigen::Vector3d a0 = Eigen::Vector3d::Random();
  a0.normalize();

  // to test geodesic distance with 0 angles and near from pi angles
  Eigen::VectorXd theta0 = Eigen::VectorXd::Random(n);
  theta0(0) = 0;
  theta0(1) = 0;
  theta0(2) = 0;
  theta0(3) = 0;
  theta0(4) = 0;
  Eigen::Matrix3Xd a = Eigen::Matrix3Xd::Random(3,n);
  a.col(0) << a0;
  a.col(1) << a0;
  a.col(2) << a0;
  a.col(4) << a0;

  double max_err = 0;

  for (size_t i=0; i<size_t(theta.size()); ++i)
  {
    Eigen::Vector3d aa = a.col(i);
    aa.normalize();
    Eigen::AngleAxisd r(theta(i), aa);
    Eigen::AngleAxisd r0(theta0(i), a0);
    max_err = std::max(max_err, t(r.matrix(), r0.matrix()));
  }

  return max_err;
}

int main(int, char **)
{
  V_TEST_LT(test(TestGeodesicDistance), 1e-7);
  V_TEST_LT(test(TestExp), 1e-7);
  V_TEST_LT(test(TestLog), 1e-1);
  V_TEST_LT(test(TestLinearInterp), 1e-7);
  V_TEST_LT(test(TestApplyRotation), 1e-7);
}
