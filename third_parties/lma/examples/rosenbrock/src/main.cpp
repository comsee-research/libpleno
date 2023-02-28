//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================

#include <libv/lma/lma.hpp>

using namespace lma;

int main()
{
  auto functor = [&](double x, double y, double& r){ r = x*x + 100.*(y - x*x)*(y - x*x); return true; };
  // Initial solution
  double x[2] ={-1,-1};
  std::cout << " Initial parameters   : (" << x[0] << "," << x[1] << ") " <<  std::endl;
  Solver<decltype(functor)>()
  .add(functor,&x[0],&x[1])
  .solve(DENSE,enable_verbose_output());
  std::cout << " Optimized parameters : (" << x[0] << "," << x[1] << ") " <<  std::endl;
  return EXIT_SUCCESS;
}
