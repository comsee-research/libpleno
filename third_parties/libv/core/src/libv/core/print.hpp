/**

\file
\author Stéphane Witzmann (2014)
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

#ifndef LIBV_CORE_PRINT_HPP
#define LIBV_CORE_PRINT_HPP

#include <iostream>
#include <sstream>
#include <utility>

namespace v {
namespace core {

  // For internal use only
  template<typename Stream>
  void print_sub(Stream &s)
  {
    s << std::endl;
  }

  // For internal use only
  template<typename Stream, typename T, typename... Args>
  void print_sub(Stream &s, T &&t, Args &&... args)
  {
    s << std::forward<T>(t);
    print_sub(s, std::forward<Args>(args)...);
  }

  /*
   * Thread-safe console/stream output.
   *
   * Concatenates printable elements into a buffer before
   * atomically printing/streaming it.
   */
  template<typename Stream, typename... Args>
  void print(Stream &s, Args &&... args)
  {
    // Build buffer
    std::stringstream tmp;
    print_sub(tmp, std::forward<Args>(args)...);

    // Then print all at once
    s << tmp.str();
  }

}
}

#endif
