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

#include <algorithm>
#include <fstream>
#include <map>
#include <stdexcept>

#include "path.hpp"

namespace v {
namespace core {

/**

Initialize a path from a string.

\param string A list of directories separated by colons.
  Example: \c "/usr/local/bin:/usr/bin:/bin"

*/
Path::Path(const std::string &string)
{
  for(std::string::const_iterator begin = string.begin(), end = string.end(), next; begin < end; begin = next + 1)
  {
    next = std::find(begin, end, ':');
    prefixes_.push_back(std::string(begin, next));
  }
}

/**

Find a file in this path.

\return The first matching file.
\throw std::runtime_error if the file was not found.

*/
std::string
Path::find(const std::string &suffix) const
{
  if(!suffix.empty() && suffix[0] == '/') return suffix;
  for(std::vector<std::string>::const_iterator prefix = prefixes_.begin(); prefix < prefixes_.end(); ++prefix)
  {
    const std::string file = *prefix + '/' + suffix;
    if(std::ifstream(file.c_str())) return file;
  }
  throw std::runtime_error("file not found: " + suffix);
}

/**

Get a path defined in an environment variable.

\param key The name of an environment variable.
\return A reference on a path.

*/
LIBV_CORE_EXPORT Path &
get_path(const std::string &key)
{
  static std::map<std::string, Path> cache;
  const std::map<std::string, Path>::iterator result = cache.find(key);
  if(result != cache.end()) return result->second;
  if(const char *value = getenv(key.c_str())) return cache[key] = Path(value);
  return cache[key];
}

LIBV_CORE_EXPORT std::string find_file(const std::string &file)
{
  static Path &path = get_path("LIBV_DATA_DIRS");
  return path.find(file);
}

}}
