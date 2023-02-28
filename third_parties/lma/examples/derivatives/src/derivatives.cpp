//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================

#include <libv/lma/lma.hpp>

using namespace lma;

auto rosenbrock = [](auto& x, auto& y, auto& r){ r = (1.-x)*(1.-x) + 100.*(y - x*x)*(y - x*x); return true; };

template<class F> int test()
{
  Eigen::Vector2d xy({-1.2,1.0});
  Solver<F>(-1,3).add(F(),&xy).solve(DENSE,enable_verbose_output());
  return EXIT_SUCCESS;
}


struct F1 // NumericCentral
{
  bool operator()(const Eigen::Vector2d& xy, double &r) const
  {
    return rosenbrock(xy[0],xy[1],r);
  }
};


struct F2 : NumericCentral
{
  bool operator()(const Eigen::Vector2d& xy, double &r) const
  {
    return rosenbrock(xy[0],xy[1],r);
  }
};


struct F3 : NumericForward
{
  bool operator()(const Eigen::Vector2d& xy, double &r) const
  {
    return rosenbrock(xy[0],xy[1],r);
  }
};


struct F4 : Analytical
{
  bool operator()(const Eigen::Vector2d& xy, double &r) const
  {
    return rosenbrock(xy[0],xy[1],r);
  }

  template<class Mat>
  void analytical_derivative(const Eigen::Vector2d& xy, Mat& mat) const
  {
    const double& x = xy[0];
    const double& y = xy[1];
    mat(0) = -2.0 * (1.0 - x) - 200.0 * (y - x * x) * 2.0 * x;
    mat(1) = 200.0 * (y - x * x);
  }
};


struct F5 : Automatic
{
  bool operator()(const Eigen::Vector2d& xy, double &r) const
  {
    return rosenbrock(xy[0],xy[1],r);
  }

  template<class AD>
  bool automatic(const Eigen::Vector2d&, const AD xy[2], AD res[1]) const
  {
    return rosenbrock(xy[0],xy[1],res[0]);
  }
};


int main()
{
  test<F1>();
  test<F2>();
  test<F3>();
  test<F4>();
  test<F5>();

  return EXIT_SUCCESS;
}