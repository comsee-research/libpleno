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

#ifndef LIBV_CORE_TYPE_TRAITS_RAW_TYPE_HPP
#define LIBV_CORE_TYPE_TRAITS_RAW_TYPE_HPP

#include <boost/mpl/identity.hpp>

#include "../global.hpp"

namespace v {
namespace core {
/// \addtogroup type_traits
/// \{

/// Remove all pointers, references and qualifiers from a type.
template<class T>
struct raw_type: boost::mpl::identity<T> {};

/// \copydoc raw_type
template<class T>
struct raw_type<T *>: boost::mpl::identity<typename raw_type<T>::type> {};

/// \copydoc raw_type
template<class T>
struct raw_type<T &>: boost::mpl::identity<typename raw_type<T>::type> {};

/// \copydoc raw_type
template<class T>
struct raw_type<const T>: boost::mpl::identity<typename raw_type<T>::type> {};

/// \}
}}

#endif
