#include <libv/lma/lma.hpp>

#include "modules.hpp"

namespace {

typedef Matrix<double, 9, 1> Vector9d;

struct _Error: Error
{
  bool operator()(const Vector9d &, double (&)[2]) const
  {
    throw this;
  }
};

typedef Solver<_Error> Solver;

struct _Module: Module
{
  Vector9d result;
  Solver solver;

  _Module()
  : solver(config.lambda(), config.iteration_count())
  {
    result << config.focal(), config.center(), config.distortion();
  }

  void init(Error &e)
  {
    Array2d a = ( e.p2 - config.center() ) / config.focal();
    double t = a.matrix().squaredNorm();
    double b = 0;
    for(size_t i = 5; i; --i) b = ( config.distortion()[i - 1] + b ) * t;

    e.p3 << a * (b + 1), 1;
  }

  void add(Error &e)
  {
    solver.add(static_cast<_Error &>(e), &result);
  }

  void run()
  {
    solver.solve(DENSE);
    cout << setprecision(20) << result.transpose() << endl;
  }

  void run_verbose()
  {
    solver.solve(DENSE,Callbacks());
    cout << boost::format("focal: [%f, %f], center: [%f, %f], distortion: [%f, %f, %f, %f, %f]") % result[0] % result[1] % result[2] % result[3] % result[4] % result[5] % result[6] % result[7] % result[8] << endl;
  }
};

}

Module *init_pinhole_indirect_radial()
{
  return new _Module;
}
