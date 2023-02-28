#include <libv/lma/lma.hpp>

// PROBLEM DEFINITION

using namespace lma;

struct Parameters{ double x,y;};

std::ostream& operator<<(std::ostream& o, const Parameters& p)
{
  return o << "Parameters(" << p.x << "," << p.y << ")";
}


struct Reprojection : lma::Analytical
{
  bool operator()(const Parameters& p, double& r) const
  {
    auto& x = p.x;
    auto& y = p.y;
    r = (1.0 - x) * (1.0 - x) + 100.0 * (y - x * x) * (y - x * x);
    return true;
  }

  template<typename Mat>
  void analytical_derivative(const Parameters& p, Mat& mat) const
  {
    auto& x = p.x;
    auto& y = p.y;
    mat[0] = -2.0 * (1.0 - x) - 200.0 * (y - x * x) * 2.0 * x;
    mat[1] = 200.0 * (y - x * x);
  }
};


namespace lma
{
  template<> struct Size<Parameters> { enum { value = 2 }; };

  void apply_increment(Parameters& parameters, const double delta[2], const Adl&)
  {
    parameters.x += delta[0];
    parameters.y += delta[1];
    //std::cout << " parameters : " << parameters.x << ", " << parameters.y << std::endl;
  }
}






int main()
{
  Parameters parameters {0.9,0.9};
  Solver<Reprojection> solver(1,15);
  solver.add(Reprojection{},&parameters);
  
  solver.solve(lma::DENSE,lma::enable_verbose_output());
}
