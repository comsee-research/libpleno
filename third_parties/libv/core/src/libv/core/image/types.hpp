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

#ifndef LIBV_CORE_IMAGE_TYPES_HPP
#define LIBV_CORE_IMAGE_TYPES_HPP

#include <libv/core/cstdint.hpp>
#include <libv/core/global.hpp>

namespace v {
namespace core {
/// \addtogroup image
/// \{

/// Define typedefs for a given image format.
#define _DEFINE_TYPE(_name, _scalar, _flags, _planes)\
  /** An \ref _name image. */\
  typedef Image<ImageFormat<_scalar, _flags, 0, 0, _planes, -1, -1> > Image##_name;\
  /** A view on an \ref _name image. */\
  typedef Image<ImageFormat<_scalar, _flags | IMAGE_SEMANTIC_POINTER, 0, 0, _planes, 0, -1> > Image##_name##p;\
  /** A view on a constant \ref _name image. */\
  typedef Image<ImageFormat<const _scalar, _flags | IMAGE_SEMANTIC_POINTER, 0, 0, _planes, 0, -1> > Image##_name##cp;\
  /** A proxy on an \ref _name image. */\
  typedef Image<ImageFormat<_scalar, _flags | IMAGE_SEMANTIC_REFERENCE, 0, 0, _planes, 0, -1> > Image##_name##r;\
  /** A proxy on a constant \ref _name image. */\
  typedef Image<ImageFormat<const _scalar, _flags | IMAGE_SEMANTIC_REFERENCE, 0, 0, _planes, 0, -1> > Image##_name##cr;\
  /** A pixel of an \ref _name image. */\
  typedef Image<ImageFormat<_scalar, _flags, 1, 1, _planes, -1, -1> > _name;\

/// The semantic of an image.
enum ImageSemantic
{
  /// The image is responsible for its contents.
  /// The memory is allocated by the constructor and deallocated by the destructor.
  /// Assigning an image changes its contents.
  /// A constant image provides only constant accessors.
  IMAGE_SEMANTIC_VALUE = 0,
  /// The image merely provides a view on an image stored somewhere else.
  /// The memory is management is left to the user.
  /// Assigning a view changes only the header of the view, not the image it points to nor its contents.
  /// A constant view provides mutable accessors.
  IMAGE_SEMANTIC_POINTER = 1,
  /// The image merely provides a proxy on an image stored somewhere else.
  /// The memory is management is left to the user.
  /// Assigning a proxy changes the contents of the image it points to.
  /// A constant proxy provides mutable accessors.
  IMAGE_SEMANTIC_REFERENCE = 2,
  /// Bits 0 to 1
  IMAGE_SEMANTIC_MASK = 3
};

/// The layout of an image.
enum ImageLayout
{
   /// The image is just an array and has no particular layout.
  IMAGE_LAYOUT_NONE = 0 << 2,
  /// Red, green, blue, opacity.
  IMAGE_LAYOUT_RGBA = 1 << 2,
  /// Blue, green, red, opacity.
  /// OpenCV and Qt prefer this layout over RGBA.
  IMAGE_LAYOUT_BGRA = 2 << 2,
  /// The image is a vector field.
  IMAGE_LAYOUT_XYZT = 3 << 2,
  /// Hue, saturation, value.
  IMAGE_LAYOUT_HSV = 4 << 2,
  /// Bits 2 to 4
  IMAGE_LAYOUT_MASK = 7 << 2
};

template<class scalar_type, int flags, int rows, int columns, int planes, int row_step, int column_step> struct ImageFormat;
template<class format> class Image;

/// \}
/// \addtogroup image_typedefs
/// \{

_DEFINE_TYPE(U8, uint8_t, 0, 1)
_DEFINE_TYPE(U16, uint16_t, 0, 1)
_DEFINE_TYPE(V3U8, uint8_t, 0, 3)
_DEFINE_TYPE(V4U8, uint8_t, 0, 4)
_DEFINE_TYPE(RGBU8, uint8_t, IMAGE_LAYOUT_RGBA, 3)
_DEFINE_TYPE(RGBAU8, uint8_t, IMAGE_LAYOUT_RGBA, 4)
_DEFINE_TYPE(BGRAU8, uint8_t, IMAGE_LAYOUT_BGRA, 4)
_DEFINE_TYPE(XYU32, uint32_t, IMAGE_LAYOUT_XYZT, 2)
_DEFINE_TYPE(XYZU32, uint32_t, IMAGE_LAYOUT_XYZT, 3)

/// \}
}}

#undef _DEFINE_TYPE
#endif
