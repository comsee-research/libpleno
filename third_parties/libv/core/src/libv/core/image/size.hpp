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

#ifndef LIBV_CORE_IMAGE_SIZE_HPP
#define LIBV_CORE_IMAGE_SIZE_HPP

#include <cstddef>

namespace v {
namespace core {
namespace image_ {

/// An optional size of an image.
template<int id, int value>
struct image_size
{
  /// Read access to this size.
  /// \return This size.
  static size_t
  get(void)
  {
    return value;
  }

  /// Write access to this size.
  static void
  set(size_t)
  {
  }

  /// Swap this size with another size.
  /// \param other An other size.
  template<class X> static void
  swap(X &other)
  {
    other.set(get());
  }
};

/// \copydoc image_size
template<int id>
struct image_size<id, 0>
{
  /// \copydoc image_size::get
  size_t
  get(void) const
  {
    return value;
  }

  /// \copydoc image_size::set
  /// \param new_value A new value for this size.
  void
  set(size_t new_value)
  {
    value = new_value;
  }

  /// \copydoc image_size::swap
  template<class X> void
  swap(X &other)
  {
    size_t x = value;
    set(other.get());
    other.set(x);
  }

private:

  size_t value;
};

}}}

#endif
