/**

\file
Compatibility with compilers that do not provide \c \<cstdint>.
Boost provide this header, but all symbols are in the \c boost namespace.
Here we just import those in the global namespace.
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

#ifndef LIBV_CORE_CSTDINT_HPP
#define LIBV_CORE_CSTDINT_HPP

#include <boost/cstdint.hpp> // for BOOST_BYTE_ORDER
#include <cstddef> // because it defines integer types

#include "tr1.hpp"

#if defined(_MSC_VER) && _MSC_VER < 1600
  namespace v {
  namespace core {
  namespace cstdint_ {
  #undef BOOST_CSTDINT_HPP // remove only this guard
  #include <boost/cstdint.hpp> // include only this file, not its dependencies
  }}}
  using namespace v::core::cstdint_::boost;
#else
  #include LIBV_INCLUDE_TR1 (cstdint)
#endif

#endif
