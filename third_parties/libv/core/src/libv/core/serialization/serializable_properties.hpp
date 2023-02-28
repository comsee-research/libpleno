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

#ifndef LIBV_CORE_SERIALIZATION_SERIALIZABLE_PROPERTIES_HPP
#define LIBV_CORE_SERIALIZATION_SERIALIZABLE_PROPERTIES_HPP

#include <libv/core/properties.hpp>

#include "serializable.hpp"

namespace v {
namespace core {
/// \addtogroup serialization
/// \{

/**

Turn a class with properties into a serializable class.
You probably don't want to use this class directly.
Use #V_DEFINE_PROPERTIES or #V_DEFINE_CONFIG instead.

*/
template<class T>
struct SerializableProperties
: virtual Serializable
{
  void serialize(InputArchive &archive)
  {
    call(&Property<T>::load, static_cast<T *>(this), archive);
  }

  void serialize(OutputArchive &archive) const
  {
    call(&Property<T>::save, static_cast<const T *>(this), archive);
  }

private:

  template<class Function, class Object, class Archive>
  static void call(Function Property<T>::*function, Object *object, Archive &archive)
  {
    for(typename std::map<std::string, Property<T> >::const_iterator p = object->properties().begin(); p != object->properties().end(); ++p)
    {
      (object->*(p->second.*function))(archive);
    }
  }
};

/// \}
/// \addtogroup config
/// \{

/**

Define a class with introspection capabilities.

\param _name The name of the class.

*/
#define V_DEFINE_PROPERTIES(_name)\
\
  struct _name: v::SerializableProperties<_name>\
  {\
    V_ENABLE_PROPERTIES(_name)\
    V_DEFINE_PROPERTIES_END

/**

Helper macro for V_DEFINE_PROPERTIES.

\param _properties The members of the class.

*/
#define V_DEFINE_PROPERTIES_END(_properties)\
\
    _properties\
  };\

/**

Define a \e Config class with introspection capabilities.

*/
#define V_DEFINE_CONFIG /** Parameters */ V_DEFINE_PROPERTIES(Config)

/// \}
}}

#endif
