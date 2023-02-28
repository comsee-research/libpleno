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

#ifndef LIBV_CORE_ITERATOR_ITERATOR_TRAITS_HPP
#define LIBV_CORE_ITERATOR_ITERATOR_TRAITS_HPP

#include <iterator>
#include <libv/core/type_traits/has_type.hpp>

namespace v {
namespace core {
/// \addtogroup type_traits
/// \{

/// Just like \c std::iterator_traits, but better.
/// More specifically, it works with \c std::back_insert_iterator and other iterators that define \c container_type.
template<class T, bool = has_container_type<T>::value>
struct iterator_traits: std::iterator_traits<T> {};

/// \copydoc iterator_traits
template<class T>
struct iterator_traits<T, true>: std::iterator_traits<typename T::container_type::iterator> {};

/// \}
}}

#endif
