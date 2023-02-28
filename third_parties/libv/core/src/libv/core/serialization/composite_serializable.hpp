/**

\file
\author Alexis Wilhelm, Stéphane Witzmann (2013, 2015, 2017)
\copyright 2013, 2015, 2017 Institut Pascal

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

#ifndef LIBV_CORE_SERIALIZATION_COMPOSITE_SERIALIZABLE_HPP
#define LIBV_CORE_SERIALIZATION_COMPOSITE_SERIALIZABLE_HPP

#include "serializable.hpp"

namespace v {
namespace core {
/// \addtogroup serialization
/// \{

/**
 * Blends two serializables into one
 */
template<typename ... Args>
struct CompositeSerializable;

template<>
struct CompositeSerializable<>
{
  void serialize(v::InputArchive &)
  {
  }

  void serialize(v::OutputArchive &) const
  {
  }
};

// Note: it may look like CompositeSerializable may work without this.
// I thought so until the compiler said there was an ambigous call in the
// recursive implementation (special case). Adding this fixed it.
template<typename T>
struct CompositeSerializable<T>: T
{
  void serialize(v::InputArchive &archive)
  {
    T::serialize(archive);
  }

  void serialize(v::OutputArchive &archive) const
  {
    T::serialize(archive);
  }
};

template<typename T, typename ... Args>
struct CompositeSerializable<T, Args...>: T, CompositeSerializable<Args...>
{
  void serialize(v::InputArchive &archive)
  {
    T::serialize(archive);
    CompositeSerializable<Args...>::serialize(archive);
  }

  void serialize(v::OutputArchive &archive) const
  {
    T::serialize(archive);
    CompositeSerializable<Args...>::serialize(archive);
  }
};

/// \}
}}

#endif
