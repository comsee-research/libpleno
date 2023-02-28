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

#ifndef LIBV_CORE_ALGORITHM_COPY_IF_HPP
#define LIBV_CORE_ALGORITHM_COPY_IF_HPP

#include <algorithm>
#include <functional>

#include "../global.hpp"

namespace v {
namespace core {
/// \addtogroup algorithms
/// \{

/**

Copy elements that satisfy a predicate into another container.

\param in An input range.
\param out An output iterator.
\param predicate A predicate.

*/
template<class T1, class T2, class F> void
copy_if(T1 &in, const T2 &out, const F &predicate)
{
  std::remove_copy_if(in.begin(), in.end(), out, bind(std::logical_not<bool>(), predicate));
}

/// \}
}}

#endif
