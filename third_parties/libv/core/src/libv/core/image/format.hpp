/**

\file
\author Alexis Wilhelm (2013)
\copyright 2013 Institut Pascal

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

#ifndef LIBV_CORE_IMAGE_FORMAT_HPP
#define LIBV_CORE_IMAGE_FORMAT_HPP

#include "types.hpp"

namespace v {
namespace core {
/// \addtogroup image
/// \{

/// The format of an Image.
/// This class encapsulates the template parameters used in the Image class, so that the Image class only takes one template parameter.
template<class _scalar_type, int _flags, int _row_count, int _column_count, int _plane_count, int _row_step, int _column_step>
struct ImageFormat
{
  static const int
    /// The row count at compile time.
    row_count = _row_count,
    /// The column count at compile time.
    column_count = _column_count,
    /// The plane count at compile time.
    plane_count = _plane_count,
    /// The row step at compile time.
    row_step = _row_step,
    /// The column step at compile time.
    column_step = _column_step,
    /// The flags at compile time.
    flags = _flags,
    /// The flags at compile time, without the semantic-related ones.
    naked_flags = flags & ~IMAGE_SEMANTIC_MASK,
    /// True if the image has value semantic.
    has_value_semantic = (flags & IMAGE_SEMANTIC_MASK) == IMAGE_SEMANTIC_VALUE,
    /// True if the image has pointer semantic.
    has_pointer_semantic = (flags & IMAGE_SEMANTIC_MASK) == IMAGE_SEMANTIC_POINTER,
    /// True if the image has reference semantic.
    has_reference_semantic = (flags & IMAGE_SEMANTIC_MASK) == IMAGE_SEMANTIC_REFERENCE,
    /// The byte count at compile time.
    byte_count = row_count * (row_step < 0 ? column_count : row_step) * plane_count * has_value_semantic,
    /// The dimension.
    dimension = (row_count != 1) + (column_count != 1) + (plane_count != 1);

  /// \copydoc Image::scalar_type
  typedef _scalar_type scalar_type;

  /// \copydoc Image::with_scalar_type
  template<class _new_scalar_type>
  struct with_scalar_type
  {
    /// \copydoc Image::with_scalar_type::type
    typedef ImageFormat<_new_scalar_type, flags, row_count, column_count, plane_count, row_step, column_step> type;
  };
};

/// \}
}}

#endif
