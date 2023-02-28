/**

\example

Check that v::core::Image can be constructed, destructed and cropped.

\author Alexis Wilhelm (2013)

\privatesection

*/

#undef NDEBUG

#include <cassert>
#include <libv/core/image/image.hpp>

using namespace v;

int
main(void)
{
  ImageU16 image(10, 100);
  V_FOR_EACH_SCALAR(p, image)
  {
    *p = uint16_t(100 * p.row + p.column);
  }

  ImageU16 copy = image;
  assert(copy == image);

  const ImageU16cp ref = image.pixels(2, 20, 5, 50);

  copy.crop(2, 20, 5, 50);
  assert(copy == ref);
  copy.crop(-2, -20, 10, 100);
  assert(copy.pixels(2, 20, 5, 50) == ref);
}
