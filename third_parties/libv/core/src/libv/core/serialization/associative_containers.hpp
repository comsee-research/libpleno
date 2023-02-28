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

#ifndef LIBV_CORE_SERIALIZATION_ASSOCIATIVE_CONTAINERS_HPP
#define LIBV_CORE_SERIALIZATION_ASSOCIATIVE_CONTAINERS_HPP

#include <map>
#include <set>

#include "archives/base.hpp"

namespace v {
namespace core {
/// \addtogroup serialization_stl
/// \{

/**

Load a pair from an input archive.

*/
template<class K, class V>
void load
( InputArchive &archive ///< An archive.
, std::pair<K, V> &value ///< A reference to a pair.
)
{
  archive
  ("first", value.first)
  ("second", value.second)
  ;
}

/**

Save a pair in an output archive.

*/
template<class K, class V>
void save
( OutputArchive &archive ///< An archive.
, const std::pair<K, V> &value ///< A pair.
)
{
  archive
  ("first", value.first)
  ("second", value.second)
  ;
}

/**

Load an associative container from an input archive.

*/
template<class T>
void load_associative_container
( InputArchive &archive ///< An archive.
, T &value ///< A container.
)
{
  value.clear();
  size_t size;
  archive.get_extents(&size, 1);
  archive.begin_list();
  for(size_t i = 0; i < size; ++i)
  {
    std::pair<typename T::key_type, typename T::mapped_type> p;
    archive(p);
    value.insert(p);
  }
  archive.end_list();
}

/// \copydoc load_associative_container()
template<class T>
void load_associative_container_set
( InputArchive &archive ///< An archive.
, T &value ///< A container.
)
{
  value.clear();
  size_t size;
  archive.get_extents(&size, 1);
  archive.begin_list();
  for(size_t i = 0; i < size; ++i)
  {
    typename T::value_type x;
    archive(x);
    value.insert(x);
  }
  archive.end_list();
}

/**

Save an associative container in an output archive.

*/
template<class T>
void save_associative_container
( OutputArchive &archive ///< An archive.
, T begin ///< An iterator to the begining of the container.
, size_t size ///< The size of the container.
)
{
  save_range(archive, begin, size);
}

/// \copydoc load_associative_container()
template<class T>
void load_associative_container_string
( InputArchive &archive ///< An archive.
, T &value ///< A container.
)
{
  value.clear();
  size_t size;
  archive.get_extents(&size, 1);
  archive.begin_list();
  for(size_t i = 0; i < size; ++i)
  {
    std::pair<std::string, typename T::mapped_type> p;
    archive(&p.first, p.second);
    value.insert(p);
  }
  archive.end_list();
}

/// \copydoc save_associative_container()
template<class T>
void save_associative_container_string
( OutputArchive &archive ///< An archive.
, T begin ///< An iterator to the begining of the container.
, size_t size ///< The size of the container.
)
{
  archive.begin_list(size);
  for(; size; --size, ++begin) archive(begin->first, begin->second);
  archive.end_list();
}

/// \copydoc load_associative_container()
template<class K, class V, class C, class A>
void load
( InputArchive &archive ///< An archive.
, std::map<K, V, C, A> &value ///< A container.
)
{
  load_associative_container(archive, value);
}

/// \copydoc save_associative_container()
template<class K, class V, class C, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::map<K, V, C, A> &value ///< A container.
)
{
  save_associative_container(archive, value.begin(), value.size());
}

/// \copydoc load_associative_container()
template<class K, class V, class C, class A>
void load
( InputArchive &archive ///< An archive.
, std::multimap<K, V, C, A> &value ///< A container.
)
{
  load_associative_container(archive, value);
}

/// \copydoc save_associative_container()
template<class K, class V, class C, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::multimap<K, V, C, A> &value ///< A container.
)
{
  save_associative_container(archive, value.begin(), value.size());
}

/// \copydoc load_associative_container()
template<class V, class C, class A>
void load
( InputArchive &archive ///< An archive.
, std::map<std::string, V, C, A> &value ///< A container.
)
{
  load_associative_container_string(archive, value);
}

/// \copydoc save_associative_container()
template<class V, class C, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::map<std::string, V, C, A> &value ///< A container.
)
{
  save_associative_container_string(archive, value.begin(), value.size());
}

/// \copydoc load_associative_container()
template<class V, class C, class A>
void load
( InputArchive &archive ///< An archive.
, std::multimap<std::string, V, C, A> &value ///< A container.
)
{
  load_associative_container_string(archive, value);
}

/// \copydoc save_associative_container()
template<class V, class C, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::multimap<std::string, V, C, A> &value ///< A container.
)
{
  save_associative_container_string(archive, value.begin(), value.size());
}

/// \copydoc load_associative_container()
template<class T, class C, class A>
void load
( InputArchive &archive ///< An archive.
, std::set<T, C, A> &value ///< A container.
)
{
  load_associative_container_set(archive, value);
}

/// \copydoc save_associative_container()
template<class T, class C, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::set<T, C, A> &value ///< A container.
)
{
  save_associative_container(archive, value.begin(), value.size());
}

/// \copydoc load_associative_container()
template<class T, class C, class A>
void load
( InputArchive &archive ///< An archive.
, std::multiset<T, C, A> &value ///< A container.
)
{
  load_associative_container_set(archive, value);
}

/// \copydoc save_associative_container()
template<class T, class C, class A>
void save
( OutputArchive &archive ///< An archive.
, const std::multiset<T, C, A> &value ///< A container.
)
{
  save_associative_container(archive, value.begin(), value.size());
}

/// \}
}}

#endif
