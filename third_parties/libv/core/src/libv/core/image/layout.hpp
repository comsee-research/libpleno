/**

\file
\author Alexis Wilhelm (2012-2013)
\copyright 2012-2013 Institut Pascal

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef LIBV_CORE_IMAGE_LAYOUT_HPP
#define LIBV_CORE_IMAGE_LAYOUT_HPP

#include "types.hpp"

/// Helper macro for #_DEFINE_LAYOUT.
#define _DEFINE_LAYOUT_(name, index)\
\
  /** Read-write access to the \e name component. */\
  /** \return A reference to \e name. */\
  value_type &\
  name(void)\
  {\
    return static_cast<this_type &>(*this)[index];\
  }\
\
  /** Read-only access to the \e name component. */\
  /** \return A const reference to \e name. */\
  const_reference\
  name(void) const\
  {\
    return static_cast<const this_type &>(*this)[index];\
  }\
\
  /** Write-only access to the \e name component. */\
  /** \param value The new value. */\
  /** \return A reference to \e this. */\
  this_type &\
  name(value_type value)\
  {\
    static_cast<this_type &>(*this)[index] = value;\
    return static_cast<this_type &>(*this);\
  }\

/// Helper macro for #_DEFINE_LAYOUT.
#define _DEFINE_LAYOUT_2(a, b)\
  _DEFINE_LAYOUT_(b, 1)\
  _DEFINE_LAYOUT_(a, 0)\
  };

/// Helper macro for #_DEFINE_LAYOUT.
#define _DEFINE_LAYOUT_3(a, b, c)\
  _DEFINE_LAYOUT_(c, 2)\
  _DEFINE_LAYOUT_2(a, b)\

/// Helper macro for #_DEFINE_LAYOUT.
#define _DEFINE_LAYOUT_4(a, b, c, d)\
  _DEFINE_LAYOUT_(d, 3)\
  _DEFINE_LAYOUT_3(a, b, c)\

/// Define an image layout.
#define _DEFINE_LAYOUT(name, size)\
  /** For images with an name layout. */\
  template<class this_type, class value_type, class const_reference>\
  struct image_layout<this_type, value_type, const_reference, IMAGE_LAYOUT_##name, size, true>\
  {\
    _DEFINE_LAYOUT_##size

namespace v {
namespace core {
namespace image_ {

template<class this_type, class value_type, class const_reference, int flags, int planes, bool enabled> struct image_layout;

/// For images with more than one pixel.
template<class this_type, class value_type, class const_reference, int flags, int planes>
struct image_layout<this_type, value_type, const_reference, flags, planes, false> {};

/// For images with no particular layout.
template<class this_type, class value_type, class const_reference, int planes>
struct image_layout<this_type, value_type, const_reference, IMAGE_LAYOUT_NONE, planes, true> {};

_DEFINE_LAYOUT(BGRA, 3)(b, g, r)
_DEFINE_LAYOUT(BGRA, 4)(b, g, r, a)
_DEFINE_LAYOUT(HSV, 3)(h, s, v)
_DEFINE_LAYOUT(RGBA, 3)(r, g, b)
_DEFINE_LAYOUT(RGBA, 4)(r, g, b, a)
_DEFINE_LAYOUT(XYZT, 2)(x, y)
_DEFINE_LAYOUT(XYZT, 3)(x, y, z)
_DEFINE_LAYOUT(XYZT, 4)(x, y, z, t)

}}}

#undef _DEFINE_LAYOUT
#undef _DEFINE_LAYOUT_
#undef _DEFINE_LAYOUT_2
#undef _DEFINE_LAYOUT_3
#undef _DEFINE_LAYOUT_4
#endif
