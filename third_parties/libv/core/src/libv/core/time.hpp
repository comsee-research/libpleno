/**

\file
Time-related functions.
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

#ifndef LIBV_CORE_TIME_HPP
#define LIBV_CORE_TIME_HPP

#include <thread>
#include <chrono>

#include "global.hpp"

namespace v {
namespace core {
/// \addtogroup time
/// \{

/**

Allow a OS compatibility function to sleep in milli-seconds.

\param duration The time to wait (in ms).

*/
inline void tempo(size_t duration)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(duration));
}

/**

Get the time at a given spot.
This function is very usefull to compute the time between two spots.
Example: \code
  double t = now();
  //... do some stuff ...
  cout << "Time elapsed : " << now()-t << " seconds" << endl;
\endcode

\return double : the time in seconds at a given spot

*/
inline double now(void)
{
  return double(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()) * 1e-6;
}

/// \}
}}

#endif
