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

#ifndef LIBV_CORE_SERIALIZATION_TYPES_HPP
#define LIBV_CORE_SERIALIZATION_TYPES_HPP

#include <libv/core/cstdint.hpp>

namespace v {
namespace core {
namespace internal_ {

template<int _rank, int _size>
struct SerializableTraits_
{
  enum {rank = _rank, size = _size};
};

}

/// \addtogroup serialization
/// \{

/**

Evaluate a macro for each fundamental type.

*/
#define V_FOR_EACH_FUNDAMENTAL_TYPE(F)\
  F(char)\
  F(int8_t)\
  F(int16_t)\
  F(int32_t)\
  F(int64_t)\
  F(uint8_t)\
  F(uint16_t)\
  F(uint32_t)\
  F(uint64_t)\
  F(float)\
  F(double)\
  F(bool)\

template<class T> struct SerializableTraits: internal_::SerializableTraits_<0, 1> {};

template<class T, int rank, int size> struct new_serializable_traits: internal_::SerializableTraits_<rank + (SerializableTraits<T>::rank && SerializableTraits<T>::size ? SerializableTraits<T>::rank : 0), size * (SerializableTraits<T>::size ? SerializableTraits<T>::size : 0)> {};

/// \}
}}

#endif
