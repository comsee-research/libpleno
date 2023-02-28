/**

\example
Convert intrinsic parameters between any two camera models.
\author Alexis Wilhelm (2013)

Currently implemented camera models are:
- Unified model
- Pinhole model with indirect radial distortions

*/

#include <libv/core/serialization/serializable.hpp>

#include "modules.hpp"

enum {UNIFIED, PINHOLE_INDIRECT_RADIAL};

Config config;
ViewerContext v1, v2, v3;

static Module *init(int type)
{
  switch(type)
  {
    case UNIFIED: return init_unified();
    case PINHOLE_INDIRECT_RADIAL: return init_pinhole_indirect_radial();
    default: abort();
  }
}

int main()
{
  load(config);
  load(config.config(), config);

  Module *input = init(config.input());
  Module *output = init(config.output());

  if(config.verbose())
  {
    pretty_print(clog, config);

    v1 = viewer()
      .size(config.width(), config.height())
      .layer(1)
      .name("Distorted points")
      .pen_width(3)
      .pen_color(red)
      ;
    v2 = viewer()
      .layer(2)
      .name("Undistorted points")
      .show(false)
      .pen_color(blue)
      ;
    v3 = viewer()
      .layer(3)
      .name("Proposed points")
      .pen_color(green)
      ;
  }
  for(double x = 0; x < config.width(); x += config.step())
  for(double y = 0; y < config.height(); y += config.step())
  {
    Error e;
    e.p2 << x, y;
    input->init(e);
    output->add(e);

    if(config.verbose())
    {
      Array2d p = e.p3.head<2>().array() * config.focal() + config.center();

      v1.add_point(e.p2.x(), e.p2.y());
      v2.add_line(e.p2.x(), e.p2.y(), p.x(), p.y());
    }
  }
  if(config.verbose())
  {
    v1.update();
    v2.update();
    output->run_verbose();
    v3.title("Done, you can now close this window");
    wait_viewers();
  }
  else
  {
    output->run();
  }
}
