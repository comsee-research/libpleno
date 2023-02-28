#include <libv/geometry/camera_model.hpp>
#include <libv/lma/lma.hpp>

#include "modules.hpp"

namespace {

struct _Error: Error
{
  bool operator()(const Vector5d &v, double (&res)[2]) const
  {
    Vector2d p;
    bool good = UnifiedCameraModel(v[0], v[1], v[2], v[3], v[4]).project(p3, p);
    if(config.verbose()) v3.add_line(p2.x(), p2.y(), p.x(), p.y());
    
    res[0] = p.x() - p2.matrix().x();
    res[1] = p.y() - p2.matrix().y();
    return good;
  }
};

typedef Solver<_Error> Solver;

struct _Module: Module
{
  Vector5d result;
  Solver solver;

  _Module()
  : solver(config.lambda(), config.iteration_count())
  {
    result << config.focal(), config.center(), config.xi();
  }

  void init(Error &e)
  {
    UnifiedCameraModel(config.focal().x(), config.focal().y(), config.center().x(), config.center().y(), config.xi()).raytrace(e.p2, e.p3);
  }

  void add(Error &e)
  {
    solver.add(static_cast<_Error &>(e),&result);
  }

  void run()
  {
    solver.solve(DENSE);
    cout << setprecision(20) << result.transpose() << endl;
  }

  void run_verbose()
  {
    solver.solve(DENSE,Callbacks());
    cout << boost::format("focal: [%f, %f], center: [%f, %f], xi: %f") % result[0] % result[1] % result[2] % result[3] % result[4] << endl;
  }
};

}

Module *init_unified()
{
  return new _Module;
}
