#include <libv/lma/lma.hpp>
#include <libv/lma/numeric/divers.hpp>
#include <libv/core/miscmath.hpp>

using namespace lma;

double square(double x) { return x*x; }

struct F1
{
  double obs;
  F1(double obs_):obs(obs_){}

  bool operator()(double x, double& res) const
  {
    res = square(x) - obs;
    return true;
  }
};

struct F2 : GermanMcClure<double>
{
  double obs;
  F2(double obs_):GermanMcClure<double>(1.48),obs(obs_){}

  bool operator()(double x, double& res) const
  {
    res = square(x) - obs;
    return true;
  }
};

int main()
{
  double x1(1);
  double x2(1);
  Solver<F1> solver1(-1,100,0.999999);
  Solver<F2> solver2(-1,100,0.999999);
  
  for(size_t i = 0 ; i < 100 ; ++i)
  {
    double d = random(0.5)-0.25;
    solver1.add(F1(d),&x1);
    solver2.add(F2(d),&x2);
  }
  
  for(size_t i = 0 ; i < 10 ; ++i)
  {
    solver2.add(F2(100+ random(100.0)),&x2);
  }

  solver1.solve(DENSE,enable_verbose_output());
  solver2.solve(DENSE,enable_verbose_output());
  std::cout << " x1 = " << x1 << std::endl;
  std::cout << " x2 = " << x2 << std::endl;
  std::cout << std::abs(x1-x2) / (x1+x2) << std::endl;
  return (std::abs(x1-x2) / (x1+x2) < 0.02 ? EXIT_SUCCESS : EXIT_FAILURE);
}
