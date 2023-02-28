/**

\example

Check that v::core::Image can be instantiated (TODO) and models the std::RandomAccessContainer concept.

\author Alexis Wilhelm (2013)

\privatesection

*/

#include <boost/concept/assert.hpp>
#include <boost/concept_check.hpp>
#include <libv/core/image/image.hpp>

using namespace v;

#define _CHECK(scalar, flags, planes)\
  _CHECK_(scalar, flags, 0, 0, planes, -1, -1)\
  _CHECK_(scalar, flags | IMAGE_SEMANTIC_POINTER, 0, 0, planes, 0, -1)\
  _CHECK_(const scalar, flags | IMAGE_SEMANTIC_POINTER, 0, 0, planes, 0, -1)\
  _CHECK_(scalar, flags | IMAGE_SEMANTIC_REFERENCE, 0, 0, planes, 0, -1)\
  _CHECK_(const scalar, flags | IMAGE_SEMANTIC_REFERENCE, 0, 0, planes, 0, -1)\
  _CHECK_(scalar, flags, 1, 1, planes, 1, 1)\

#define _CHECK_(a, b, c, d, e, f, g)\
  template struct check<Image<ImageFormat<a, b, c, d, e, f, g> > >;\

#define _ASSERT(T)\
  BOOST_CONCEPT_ASSERT((boost::RandomAccessContainer<T>));\

template<class T>
struct check
{
  _ASSERT(T)
  _ASSERT(typename T::row_type)
  _ASSERT(typename T::column_type)
  _ASSERT(typename T::reference_to_pixel)
  _ASSERT(typename T::view_type)
  _ASSERT(typename T::proxy_type)
};

_CHECK(float, 0, 4)
_CHECK(uint8_t, 0, 1)
_CHECK(uint8_t, 0, 3)
_CHECK(uint8_t, 0, 4)
_CHECK(uint8_t, IMAGE_LAYOUT_RGBA, 3)
_CHECK(uint8_t, IMAGE_LAYOUT_RGBA, 4)
_CHECK(uint32_t, IMAGE_LAYOUT_XYZT, 2)
_CHECK(uint32_t, IMAGE_LAYOUT_XYZT, 3)

int
main(void)
{
}
