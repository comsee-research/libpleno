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

#ifndef LIBV_CORE_TYPE_TRAITS_PREVENT_OVERFLOW_HPP
#define LIBV_CORE_TYPE_TRAITS_PREVENT_OVERFLOW_HPP

#include "../cstdint.hpp"

namespace v {
namespace core {
namespace prevent_overflow_ {

template<class T>
struct prevent_overflow
{
  typedef T type;
};

#define _REGISTER_OVERFLOW_TYPE(a, b)\
\
  template<>\
  struct prevent_overflow<a>\
  {\
    typedef b type;\
  };\

_REGISTER_OVERFLOW_TYPE(int8_t, int16_t)
_REGISTER_OVERFLOW_TYPE(int16_t, int32_t)
_REGISTER_OVERFLOW_TYPE(int32_t, int64_t)
_REGISTER_OVERFLOW_TYPE(uint8_t, uint16_t)
_REGISTER_OVERFLOW_TYPE(uint16_t, uint32_t)
_REGISTER_OVERFLOW_TYPE(uint32_t, uint64_t)

#undef _REGISTER_OVERFLOW_TYPE

}

/// \addtogroup type_traits
/// \{

/// A type slightly larger than \e T, so that multiplying by 4 does not cause overflow.
template<class T> struct prevent_overflow: prevent_overflow_::prevent_overflow<T> {};

/// \}
}}

#endif
