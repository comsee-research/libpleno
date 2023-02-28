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

#ifndef LIBV_CORE_TYPE_TRAITS_ENABLE_IF_HPP
#define LIBV_CORE_TYPE_TRAITS_ENABLE_IF_HPP

#include <type_traits>

#include "../macros.hpp"

/// \addtogroup polymorphism
/// \{

/// Enable a function overload only if a condition is true.
/// The point of this macro is that Doxygen will document the condition in the function's documentation rather than in its parameter list.
/// \warning You must not write the opening parenthesis of the parameter list: this macro does it for you.
#define V_ENABLE_IF(_test, _type)(typename std::conditional<(_test), _type, struct V_CONCATENATE(DISABLED_AT_LINE_, __LINE__) &>::type

/// Enable a template function overload only if a condition is true.
/// The point of this macro is that Doxygen will document the condition in the function's documentation rather than in its parameter list.
/// \warning You must not write the closing parenthesis of the parameter list: this macro does it for you.
#define V_ENABLE_IF_TPL(_test), typename std::enable_if<(_test)>::type * = nullptr)

/// \copydoc V_ENABLE_IF
/// \note For some reason #V_ENABLE_IF won't work for \c operator()(), so we need to add this special case.
#define V_OPERATOR_ENABLE_IF(_test, _type) operator() V_ENABLE_IF(_test, _type)

/// \}

#ifdef DOXYGEN
/// \cond false

#undef V_ENABLE_IF
#define V_ENABLE_IF(_test, _type) /** \pre \code _test \endcode */ (_type

#undef V_ENABLE_IF_TPL
#define V_ENABLE_IF_TPL(_test)) /** \pre \code _test \endcode */

#undef V_OPERATOR_ENABLE_IF
#define V_OPERATOR_ENABLE_IF(_test, _type) /** \pre \code _test \endcode */ operator()(_type

/// \endcond
#endif
#endif
