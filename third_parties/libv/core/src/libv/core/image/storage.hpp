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

#ifndef LIBV_CORE_IMAGE_STORAGE_HPP
#define LIBV_CORE_IMAGE_STORAGE_HPP

namespace v {
namespace core {
namespace image_ {

/// When all dimensions are known at compile time.
template<class T, int n>
struct image_storage_type
{
  /// Result
  typedef T type[n];
};

/// When some dimensions are dynamic.
template<class T>
struct image_storage_type<T, 0>
{
  /// Result
  typedef T *type;
};

}}}

#endif
