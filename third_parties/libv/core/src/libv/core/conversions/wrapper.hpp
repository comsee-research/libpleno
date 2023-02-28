/**

\file
\author Alexis Wilhelm (2012)
\copyright 2012 Institut Pascal

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

#ifndef LIBV_CORE_CONVERSIONS_WRAPPER_HPP
#define LIBV_CORE_CONVERSIONS_WRAPPER_HPP

#include <libv/core/global.hpp>

namespace v {
namespace core {
/// \addtogroup conversions
/// \{

/// Wrap the contents of a container in another kind of container.
/// \tparam Base The destination type.
template<class Base>
struct Wrapper
  : Base
{
  /// Initialize this wrapped container with the contents of another container.
  /// Calls a free function named \c convert.
  /// Support for more conversions is achieved by overloading this function for each pair of container type.
  /// \tparam T The source type.
  /// \param src The container to be wrapped.
  template<class T> Wrapper(T &src)
    : Base(convert(src, this))
  {
  }
};

/// Holds an object convertible to \e To.
template<class To>
struct Convertible
{
  /// Convert the holded object to \e To.
  virtual To convert(void) = 0;
  virtual ~Convertible(void) {}
};

/// Holds an object of type \e From, convertible to \e To.
template<class From, class To>
struct Convertible_
  : Convertible<To>
{
  /// Constructor.
  Convertible_(const From &other)
    : data(other)
  {
  }

  To convert(void)
  {
    return Wrapper<To>(data);
  }

private:

  From data;
};

/// \}
}}

#endif
