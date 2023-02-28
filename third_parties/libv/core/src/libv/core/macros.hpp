/**

\file
\author Alexis Wilhelm (2012)
\copyright 2012 Institut Pascal

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

/// \addtogroup preprocessor
/// \{

/// Stringify the value of a macro.
/// Use this if \c \#x does not work.
#define V_STRINGIFY(x) V_STRINGIFY_(x)

/// Helper macro for #V_STRINGIFY.
#define V_STRINGIFY_(x) #x

/// Concatenate the value of two macros.
/// Use this if \c x\#\#y does not work.
#define V_CONCATENATE(x, y) V_CONCATENATE_(x, y)

/// Helper macro for #V_CONCATENATE.
#define V_CONCATENATE_(x, y) x##y

/// \}
