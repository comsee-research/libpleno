/**

\example

Check that v::core::Image has working iterators.

\author Alexis Wilhelm (2013)

\privatesection

*/

#undef NDEBUG

#include <cassert>
#include <libv/core/image/image.hpp>

using namespace v;

template<class T> void
test_iterator(const T &begin, const T &end, void (*test)(const T &, const T &, ptrdiff_t))
{
  ptrdiff_t i = 0;
  for(T p = begin; p != end; ++p, ++i)
  {
    assert(p - begin == i);
    test(begin, p, i);
  }
}

template<class T> void
test_row_iterator(const T &begin, const T &p, ptrdiff_t i)
{
  assert(p->pixel(0, 0).x() == begin->pixel(0, 0).x());
  assert(ptrdiff_t(p->pixel(0, 0).y()) == i);
}

template<class T> void
test_column_iterator(const T &begin, const T &p, ptrdiff_t i)
{
  assert(ptrdiff_t(p->pixel(0, 0).x()) == i);
  assert(p->pixel(0, 0).y() == begin->pixel(0, 0).y());
}

template<class T> void
test_pixel_iterator(const T &, const T &p, ptrdiff_t i)
{
  assert(ptrdiff_t(p->x()) == i % 100);
  assert(ptrdiff_t(p->y()) == i / 100);
}

template<class T> void
test_scalar_iterator(const T &, const T &p, ptrdiff_t i)
{
  switch(i % 2)
  {
    case 0: assert(ptrdiff_t(*p) == i / 2 % 100); break;
    case 1: assert(ptrdiff_t(*p) == i / 200); break;
  }
}

int
main(void)
{
  ImageXYU32 image(10, 100);
  V_FOR_EACH_PIXEL(p, image)
  {
    p->x() = p.column;
    p->y() = p.row;
  }

  test_iterator(image.begin_row(), image.end_row(), test_row_iterator);
  test_iterator(image.begin_column(), image.end_column(), test_column_iterator);
  test_iterator(image.begin_pixel(), image.end_pixel(), test_pixel_iterator);
  test_iterator(image.begin_scalar(), image.end_scalar(), test_scalar_iterator);
}
