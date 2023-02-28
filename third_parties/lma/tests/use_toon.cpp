/*
#define USE_TOON 1

#include <libv/lma/lma.hpp>
#include <libv/core/miscmath.hpp>

using namespace lma;

double rosenbrock(double x, double y)
{
  return v::pow<2>(1-x) + 100*v::pow<2>(y - v::pow<2>(x));
}

template<class F> void solve(F f)
{
  double x(-1),y(-1);
  Solver<F>(1,10).add(f,&x,&y).solve(TOON_DENSE,enable_verbose_output());
}

template<class R> struct Fonctor
{
  bool operator()(double x, double y, R& res) const
  {
    res[0] = rosenbrock(x,y);
    return true;
  }
};

template<> struct Fonctor<double>
{
  bool operator()(double x, double y, double& res) const
  {
    res = rosenbrock(x,y);
    return true;
  }
};
*/
int main()
{/*
  // residual : double&
  solve(Fonctor<double>());
  
  // residual : double [1]
  solve(Fonctor<double[1]>());
  
  // residual : std::array<double,1>&
  solve(Fonctor<std::array<double,1>>());
  
  //residual : TooN::Matrix<double,1,1>&
  solve(Fonctor<TooN::Vector<1,double>>());*/
}

