/**

\file
Definition of the Codec interface.
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

#ifndef LIBV_CORE_AUTO_LOAD_HPP
#define LIBV_CORE_AUTO_LOAD_HPP

#include <map>
#include <sstream>
#include <stdexcept>
#include <typeinfo>

#include "global.hpp"

namespace v {
namespace core {
namespace auto_load_ {

LIBV_CORE_EXPORT std::multimap<int, void *> *get_codecs(const char *, const char *);
LIBV_CORE_EXPORT std::string demangle(const std::type_info &);

template<class _type>
std::string type_name(_type &x)
{
  return demangle(typeid(x));
}

template<class Input, class Output>
struct Codec
{
  static void load(Input &input, Output &output)
  {
    std::ostringstream message;
    message << "No codec found for conversion from " << type_name(input) << " to " << type_name(output) << ". Excuses are:";
    for(typename List::const_reference pair: instances())
    {
      try
      {
        return (*pair.second)(input, output);
      }
      catch(std::exception &e)
      {
        message << "\n" << type_name(*pair.second) << " failed with " << type_name(e) << ": " << e.what();
      }
      catch(...)
      {
        message << "\n" << type_name(*pair.second) << " failed.";
      }
    }
    throw std::runtime_error(message.str());
  }

protected:

  typedef std::multimap<int, const Codec *> List;
  virtual void operator()(Input &input, Output &output) const = 0;

  static List &instances(void)
  {
    return *reinterpret_cast<List *>(get_codecs(typeid(Input).name(), typeid(Output).name()));
  }
};

}

/// \addtogroup io
/// \{

/**

A codec registered in the global codec list.
The codecs that extend this class are registered automatically.

\tparam priority The priority of the registered codec.
  Codecs with a higher priority are tried before those with a lower priority.

*/
template<class Input, class Output, int priority = 0>
struct register_codec
  : auto_load_::Codec<Input, Output>
{
protected:

  /// Constructor
  register_codec(void)
  {
    typedef auto_load_::Codec<Input, Output> Base;
    Base::instances().insert(typename Base::List::value_type(-priority, this));
  }
};

/**

Load an object using one of the registered codecs.

\param input The data source.
\param output The loaded object.
\throws IMPLEMENTATION_DEFINED If the object cannot be loaded.

*/
template<class Input, class Output> void
auto_load(Input &input, Output &output)
{
  auto_load_::Codec<Input, Output>::load(input, output);
}

/// \}
}}

#endif
