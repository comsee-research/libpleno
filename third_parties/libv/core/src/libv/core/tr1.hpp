/**

\file
Compatibility with compilers that do not put TR1 headers in a \c tr1 directory.
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

#include "global.hpp"

#if defined(_MSC_VER)
  #define LIBV_INCLUDE_TR1(header) <header>
#else
  #define LIBV_INCLUDE_TR1(header) <tr1/header>
#endif

#if defined __GNUC__ && !defined __GXX_EXPERIMENTAL_CXX0X__
  #define V_DECLTYPE typeof
#else
  #define V_DECLTYPE decltype
#endif
