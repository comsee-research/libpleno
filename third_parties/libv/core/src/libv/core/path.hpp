/**

\file
Find files in a list of directories.
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

#ifndef LIBV_CORE_PATH_HPP
#define LIBV_CORE_PATH_HPP

#include <string>
#include <vector>

#include "global.hpp"

namespace v {
namespace core {
/// \addtogroup config
/// \{

/**

A list of directories where configuration files are stored.

*/
struct LIBV_CORE_EXPORT Path
{
  Path(void) {}
  Path(const std::string &);
  std::string find(const std::string &) const;
private:
  std::vector<std::string> prefixes_;
};

LIBV_CORE_EXPORT Path &get_path(const std::string &);
LIBV_CORE_EXPORT std::string find_file(const std::string &);

/// \}
}}

#endif
