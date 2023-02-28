#include <libv/core/serialization/contiguous_containers.hpp>
#include <libv/core/serialization/eigen.hpp>
#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/graphic/viewer_context.hpp>
#include <libv/lma/lma.hpp>
#include <Eigen/Core>

using namespace v;
using namespace std;
using namespace Eigen;

typedef Matrix<double, 5, 1> Vector5d;

V_DEFINE_CONFIG
(
  V_DEFINE_PROPERTY(config, string("config.ini"), "Main config file")
  V_DEFINE_PROPERTY(input, 1, "The Input camera model")
  V_DEFINE_PROPERTY(output, 0, "The output camera model")
  V_DEFINE_PROPERTY(step, 10., "Distance between two consecutive generated points")
  V_DEFINE_PROPERTY(height, 480., "Height of the image")
  V_DEFINE_PROPERTY(width, 640., "Width of the image")
  V_DEFINE_PROPERTY(center, Array2d(), "Coordinates of the principal point")
  V_DEFINE_PROPERTY(focal, Array2d(), "Focal length")
  V_DEFINE_PROPERTY(xi, 0., "Distortion parameter for the unified model")
  V_DEFINE_PROPERTY(distortion, Vector5d(), "Distortion polynomial")
  V_DEFINE_PROPERTY(lambda, 1e-3, "")
  V_DEFINE_PROPERTY(iteration_count, 25u, "")
  V_DEFINE_PROPERTY(verbose, false, 0)
)

extern Config config;
extern ViewerContext v1, v2, v3;

struct Callbacks
: enable_verbose_output
{
  template<class S, class Algo>
  void at_begin_bundle_adjustment_iteration(const S& solver, const Algo& algo) const
  {
    enable_verbose_output::at_begin_bundle_adjustment_iteration(solver,algo);
    v3.clear();
  }

  template<class S, class Algo>
  void at_end_bundle_adjustment_iteration(const S& solver, const Algo& algo) const
  {
    enable_verbose_output::at_end_bundle_adjustment_iteration(solver,algo);
    v3.update().title("Press enter to continue");
    getchar();
  }
};

struct Error
{
  Array2d p2;
  Vector3d p3;
};

struct Module
{
  virtual void init(Error &) = 0;
  virtual void add(Error &) = 0;
  virtual void run() = 0;
  virtual void run_verbose() = 0;
};

Module *init_unified();
Module *init_pinhole_indirect_radial();
