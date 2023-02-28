/**

\file
\author Alexis Wilhelm (2012)
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

#ifndef LIBV_CORE_SERIALIZATION_CONTIGUOUS_CONTAINERS_HPP
#define LIBV_CORE_SERIALIZATION_CONTIGUOUS_CONTAINERS_HPP

#include <array>
#include <vector>

#include "archives/base.hpp"

namespace v {
namespace core {
/// \addtogroup serialization_stl
/// \{

/**

Load a contiguous container from an input archive.

*/
template<class T>
void load_contiguous_container
( InputArchive &archive ///< An archive.
, T &value ///< A reference to a container.
)
{
  size_t extents[SerializableTraits<T>::rank];
  archive.get_extents(extents, SerializableTraits<T>::rank);
  value.resize(extents[0]);
  load(archive, &value[0], extents, 1);
}

/**

Save a contiguous container in an output archive.

*/
template<class T>
void save_contiguous_container
( OutputArchive &archive ///< An archive.
, const T &value ///< A container.
)
{
  size_t extents[SerializableTraits<T>::rank];
  extents[0] = value.size();
  save(archive, &value[0], extents, 1);
}

template<class T, size_t n>
struct SerializableTraits<std::array<T, n>>: new_serializable_traits<T, 1, n> {};

/// \copydoc load_contiguous_container()
template<class T, size_t n>
void load
( InputArchive &archive ///< An archive.
, std::array<T, n> *values ///< A list of arrays.
, const size_t *extents ///< A list of extents.
, size_t rank ///< The rank of the array.
)
{
  load(archive, values->data(), extents, rank + 1);
}

/// \copydoc save_contiguous_container()
template<class T, size_t n>
void save
( OutputArchive &archive ///< An archive.
, const std::array<T, n> *values ///< A list of arrays.
, size_t *extents ///< A list of extents.
, size_t rank ///< The rank of the array.
)
{
  extents[rank] = values->size();
  save(archive, values->data(), extents, rank + 1);
}

template<class T, class O, class A>
struct SerializableTraits<std::basic_string<T, O, A> >: new_serializable_traits<T, 1, 0> {};

/// \copydoc load_contiguous_container()
template<class T, class O, class A>
void load
( InputArchive &archive ///< An archive.
, std::basic_string<T, O, A> &value ///< A container.
)
{
  load_contiguous_container(archive, value);
}

/// \copydoc save_contiguous_container()
template<class T, class O, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::basic_string<T, O, A> &value ///< A container.
)
{
  save_contiguous_container(archive, value);
}

template<class T, class A>
struct SerializableTraits<std::vector<T, A> >: new_serializable_traits<T, 1, 0> {};

/// \copydoc load_contiguous_container()
template<class T, class A>
void load
( InputArchive &archive ///< An archive.
, std::vector<T, A> &value ///< A container.
)
{
  load_contiguous_container(archive, value);
}

/// \copydoc save_contiguous_container()
template<class T, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::vector<T, A> &value ///< A container.
)
{
  save_contiguous_container(archive, value);
}

/// \}
}}

#endif
