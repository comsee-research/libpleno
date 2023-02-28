#include <libv/lma/lma.hpp>

struct Error : lma::CheckAnalytical
{
  bool operator()(const Eigen::Vector3d& a, const double& b, double (&error) [2]) const
  {
  	error[0] = a.x() * b + a.y()*b + a.z();
  	error[1] = (a.x() + a.y() + a.z()) * b;
  	return true;
  }

  template<typename JA, typename JB>
  void analytical_derivative(const Eigen::Vector3d&, const double&, JA& ja, JB& jb) const
  {
  	std::cout << " Jacob(a):\n " << ja << std::endl;
  	std::cout << " Jacob(b):\n " << jb << std::endl;
  }
};

using namespace lma;

int main()
{
  Eigen::Vector3d a(1,2,3);
  double b(4);
  Solver<Error>().add(Error(),&a,&b).solve(DENSE,enable_verbose_output());
}
