#include <libv/lma/lma.hpp>

using namespace lma;
using namespace Eigen;

double distance(const Vector4d& sphere, const Vector3d& point)
{
  return (sphere.head<3>() - point).norm() - sphere[3];
};


int main()
{
  struct Error
  {
    bool operator()(const Vector4d& sphere, const Vector3d& point, double& residual) const
    {
      residual = distance(sphere,point);
      return true;
    }
  };
  
  Vector4d sphere(0,0,0,100);
  Vector3d point(60,30,80);
 
  std::cout << "Error before = " << distance(sphere,point) << std::endl; 

  Solver<Error> solver;
  solver.add(Error{},&sphere,&point);
  solver.solve(DENSE_SCHUR, enable_verbose_output());

  std::cout << "Error after = " << distance(sphere,point) << std::endl; 
}