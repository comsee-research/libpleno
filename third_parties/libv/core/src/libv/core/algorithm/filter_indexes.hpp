/**

\file
\author Alexis Wilhelm (2014)
\copyright 2014 Institut Pascal

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

#ifndef LIBV_CORE_ALGORITHM_FILTER_INDEXES_HPP
#define LIBV_CORE_ALGORITHM_FILTER_INDEXES_HPP

#include <libv/core/assert.hpp>

namespace v {
namespace core {
/// \addtogroup algorithms
/// \{

/**

Filter elements by their index.

*/
template<class T, class F>
void filter_indexes
( T &m ///< A random access container.
, const F &indexes ///< A container of indexes.
)
{
  V_PRECONDITION(m.size() >= indexes.size());
  typename T::iterator p = m.begin();

  for(typename F::const_iterator index = indexes.begin(); index != indexes.end(); ++index)
  {
    *p = m[*index];
    ++p;
  }

  m.erase(p, m.end());
}

/// \}
}
}

#endif
