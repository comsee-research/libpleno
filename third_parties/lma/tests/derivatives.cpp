#include <libv/lma/lma.hpp>
#include <libv/lma/numeric/divers.hpp>
#include <libv/core/miscmath.hpp>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>

using namespace lma;


template<class Derivator>
struct F : Derivator
{
  double obs;
  F(double obs_):obs(obs_){}

  template<class X, class Y, class Res> bool error(const X& x, const Y& y, Res& res) const
  {
    res = x * y - obs;
    return true;
  }

  bool operator()(double x, double y, double& res) const
  {
    return error(x,y,res);
  }
  
  template<class Mat>
  void analytical_derivative(double x, double y, Mat& mat1, Mat& mat2) const
  {
    static_assert(boost::is_same<Analytical,Derivator>::value,"Only for analytical mode");
    mat1(0,0) = y;
    mat2(0,0) = x;
  }
  
  template<class AD>
  bool automatic(double, AD x[1], double, AD y[1], AD res[1]) const
  {
    static_assert(boost::is_same<Automatic,Derivator>::value,"Only for automatic mode");
    return error(x[0],y[0],res[0]);
  }
};

template<class F> std::tuple<double,double> solve(double y, double x, std::vector<double> obs)
{
  Solver<F> solver;
  for(double o : obs)
    solver.add(F(o),&x,&y);
  solver.solve(DENSE,minimal_verbose());
  return std::make_tuple(x,y);
}

std::ostream& operator<<(std::ostream& o, std::tuple<double,double> tuple)
{
  return o << std::get<0>(tuple) << "," << std::get<1>(tuple) << std::endl;
}

struct None{};

int main()
{
  std::vector<double> obs = {0.1,0.2,-0.05,-0.15,0.0};
  std::cout << solve<F<None>>(1,1,obs) << std::endl;
  std::cout << solve<F<NumericCentral>>(1,1,obs) << std::endl;
  std::cout << solve<F<NumericForward>>(1,1,obs) << std::endl;
  std::cout << solve<F<Analytical>>(1,1,obs) << std::endl;
  std::cout << solve<F<Automatic>>(1,1,obs) << std::endl;

  return EXIT_SUCCESS;
}
