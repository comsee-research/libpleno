/**

\file
Load and save configuration files.
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

#include <boost/property_tree/info_parser.hpp>
#include <libv/core/auto_load.hpp>
#include <libv/core/serialization/archives/property_tree.hpp>

#include "serializable.hpp"

#if defined(__posix__)
  #include <unistd.h>
#endif

using namespace boost::property_tree;

namespace v {
namespace core {
namespace {

static PropertyTreeInputArchive &environment()
{
  static ptree tree;
  static PropertyTreeInputArchive archive(tree);
  for(size_t i = 0; environ[i]; ++i)
  {
    const std::string var = environ[i];
    const size_t n = var.find('=');
    tree.put(var.substr(0, n), var.substr(n + 1));
  }
  return archive;
}

}

/**

Populate the properties of an object using the environment.

\warning Might be UNIX-specific.

*/
LIBV_CORE_EXPORT void load
( Serializable &output ///< A serializable object.
)
{
  static PropertyTreeInputArchive &archive = environment();
  v::load(archive, output);
}

/**

Load a config file.
May be INI, XML, Json or Info.

*/
LIBV_CORE_EXPORT void load
( const std::string &input ///< A config file.
, Serializable &output ///< A serializable object.
)
{
  auto_load(input, output);
}

/**

Save a config file.
The format is chosen based on the filename extension.
May be INI, XML, Json or Info.

*/
LIBV_CORE_EXPORT void save
( const std::string &output ///< A config file.
, const Serializable &input ///< A serializable object.
)
{
  auto_load(input, output);
}

/**

Pretty-print a serializable object.

*/
LIBV_CORE_EXPORT void pretty_print
( std::ostream &output ///< An output stream.
, const Serializable &input ///< A serializable object.
)
{
  ptree tree;
  PropertyTreeOutputArchive archive(tree);
  archive.default_key("*");
  v::save(archive, input);
  write_info(output, tree);
}

}}
