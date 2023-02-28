/**
\file

\author Simeon Schwab
\copyright 2014 Institut Pascal
*/

#include <cstdlib>
#include <iostream>
#include <vector>
#include <libv/geometry/plane_equation.hpp>

using namespace v;

#define LIBV_CHECK(x) if( !(x) ) { std::cerr << "CTest: Check fails in file " << __FILE__ << " at line " << __LINE__ << std::endl; return EXIT_FAILURE; }

template <typename type_>
type_ test()
{

  // equation
  Eigen::Matrix<type_, 4, 1> eq;
  eq << 1.3348,10.3215,0.002154,0;
  eq.normalize();

  eq[3] = -125.215;

  // initilize points
  std::vector<Eigen::Matrix<type_, 3, 1> > p;
  const size_t n=10000;
  p.reserve(n);

  typedef Eigen::Array<type_,Eigen::Dynamic,Eigen::Dynamic> Array;
  Array x = Array::Random(n,1);
  Array y = Array::Random(n,1);
  Array z = -(eq.w()+eq.x()*x+eq.y()*y)/eq.z();

  for (size_t i=0; i<n; ++i)
  {
    p.push_back(Eigen::Matrix<type_, 3, 1>(x(i,0),y(i,0),z(i,0)));
  }

  // solve
  PlaneEquationFrom3DPoints<type_> solver;
  Eigen::Matrix<type_, 4, 1> out = solver.compute_plane(p.begin(), p.end());

  // take the opposite equation if the d found is positive
  if (out.w()>0)
  {
    out = -out;
  }

  return (out-eq).array().abs().maxCoeff();

}

int main(int, char **)
{
  LIBV_CHECK(test<float>()<10e-4);
  LIBV_CHECK(test<double>()<10e-12);
  LIBV_CHECK(test<long double>()<10e-14);
  return EXIT_SUCCESS;
}


