/// \example
/// Draw random shapes.
/// \author Alexis Wilhelm (2013)
/// \see viewers
/// \privatesection

#include <cstdlib>
#include <libv/graphic/viewer_context.hpp>

static float
REAL(void)
{
  return float(std::rand()) / float(RAND_MAX) * float(512);
}

static const std::string &
TEXT(void)
{
  static const std::string texts[] = {
    "Lorem ipsum dolor sit amet,",
    "consectetuer adipiscing elit.",
  };
  return texts[std::rand() % 2];
}

int
main(void)
{
  v::viewer(1)
    .title("Test numéro 1")
    .layer(1).name("Rectangles")
    .layer(2).name("Lines")
    .layer(3).name("Points")
    .axes(v::AxesXY);

  v::viewer(2)
    .title("Test numéro 2")
    .layer(1).name("Arrows")
    .layer(2).name("Circles")
    .layer(3).name("Ellipses")
    .layer(4).name("Text")
    .background(v::light_yellow);

  v::viewer(3)
    .title("Test numéro 3")
    .layer(1).name("Triangles")
    .layer(2).name("Quadrilaterals")
    .size(200, 200);

  for(;;)
  {
    v::ViewerContext v11 = v::viewer(1).layer(1).clear().pen_color(v::blue);
    v::ViewerContext v12 = v::viewer(1).layer(2).clear().pen_width(2).pen_color(v::green);
    v::ViewerContext v13 = v::viewer(1).layer(3).clear().pen_width(3).pen_color(v::red);

    for(int i = 0; i < 1000; ++i)
    {
      v11.add_rect(REAL(), REAL(), REAL() / 10, REAL() / 10);
      v12.add_line(REAL(), REAL(), REAL(), REAL());
      v13.add_point(REAL(), REAL());
    }

    v11.update();
    v12.update();
    v13.update();

    v::ViewerContext v21 = v::viewer(2).layer(1).clear().pen_width(2).pen_color(v::red);
    v::ViewerContext v22 = v::viewer(2).layer(2).clear().pen_color(v::dark_green).brush_color(v::green).brush_style(v::SolidPattern);
    v::ViewerContext v23 = v::viewer(2).layer(3).clear().pen_color(v::dark_blue).brush_color(v::blue).brush_style(v::SolidPattern);
    v::ViewerContext v24 = v::viewer(2).layer(4).clear();

    for(int i = 0; i < 10; ++i)
    {
      v21.add_arrow(REAL(), REAL(), REAL(), REAL());
      v22.add_circle(REAL(), REAL(), REAL() / 10);
      v23.add_ellipse(REAL(), REAL(), REAL() / 10, REAL() / 10);
      v24.add_text(REAL(), REAL(), TEXT());
    }

    v21.update();
    v22.update();
    v23.update();
    v24.update();

    v::ViewerContext v31 = v::viewer(3).layer(1).clear().pen_color(v::dark_cyan).brush_color(v::cyan).brush_style(v::SolidPattern);
    v::ViewerContext v32 = v::viewer(3).layer(2).clear().pen_color(v::dark_magenta).brush_color(v::magenta).brush_style(v::SolidPattern);

    for(int i = 0; i < 10; ++i)
    {
      v31.add_triangle(REAL(), REAL(), REAL(), REAL(), REAL(), REAL());
      v32.add_quad(REAL(), REAL(), REAL(), REAL(), REAL(), REAL(), REAL(), REAL());
    }

    v31.update();
    v32.update();
  }
}
