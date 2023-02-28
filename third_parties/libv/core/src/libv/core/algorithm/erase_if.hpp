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

#ifndef LIBV_CORE_ALGORITHM_ERASE_IF_HPP
#define LIBV_CORE_ALGORITHM_ERASE_IF_HPP

#include <algorithm>

#include "../global.hpp"

namespace v {
namespace core {
/// \addtogroup algorithms
/// \{

/**

Erase elements that satisfy a predicate.

\param m A container.
\param predicate A predicate.

*/
template<class T, class F> void
erase_if(T &m, const F &predicate)
{
  m.erase(std::remove_if(m.begin(), m.end(), predicate), m.end());
}

/// \}
}}

#endif
