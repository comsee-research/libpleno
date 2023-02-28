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

#include <cassert>

#include "global.hpp"

/// \addtogroup logger
/// \{

/// Check that a condition is true at run time.
/// This macro behaves just like \c assert(), but the function in which you use it gets free automatic documentation as a bonus.
#define V_PRECONDITION(_test) /** \pre \code _test \endcode */ assert(_test);

/// \copydoc V_PRECONDITION
#define V_POSTCONDITION(_test) /** \post \code _test \endcode */ assert(_test);

/// \}
