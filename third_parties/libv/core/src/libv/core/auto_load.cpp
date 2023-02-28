/**

\file
\author Alexis Wilhelm (2013-2014)
\copyright 2013-2014 Institut Pascal

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

#include <libv/core/auto_load.hpp>

#if defined(__GLIBCXX__) || defined(__GLIBCPP__)
  #define _HAS_CXXABI_H
#endif
#if defined(_HAS_CXXABI_H)
  #include <cxxabi.h>
#endif

namespace v {
namespace core {
namespace auto_load_ {

std::multimap<int, void *> *get_codecs(const char *type_1, const char *type_2)
{
  static std::map<std::pair<std::string, std::string>, std::multimap<int, void *> > codecs;
  return &codecs[std::make_pair(type_1, type_2)];
}

std::string demangle(const std::type_info &type)
{

#if defined(_HAS_CXXABI_H)

  char *name = abi::__cxa_demangle(type.name(), 0, 0, 0);

  if(name)
  {
    std::string copy = name;
    free(name);
    return copy;
  }

#endif

  return type.name();
}

}
}
}
