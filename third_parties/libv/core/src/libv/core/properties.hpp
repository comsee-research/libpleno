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

#ifndef LIBV_CORE_PROPERTIES_HPP
#define LIBV_CORE_PROPERTIES_HPP

#include <libv/core/serialization/archives/base.hpp>
#include <libv/core/type_traits/raw_type.hpp>
#include <map>

namespace v {
namespace core {
/// \addtogroup config
/// \{

/**

Enable introspection capabilities for a class.
The \c properties() static method holds the list of the properties defined with #V_DEFINE_PROPERTY.

\param class The name of the class.

\post The class contains the following new members: \code

  typedef this_type; // the named class
  static std::map<std::string, Property> &properties(void); // the properties defined in this class
  this_type copy() const; // a copy of this object

\endcode

*/
#define V_ENABLE_PROPERTIES(class)\
\
  typedef class this_type;\
\
  static std::map<std::string, v::Property<class> > &properties(void)\
  {\
    static std::map<std::string, v::Property<class> > value;\
    return value;\
  }\
\
  this_type copy() const\
  {\
    return *this;\
  }\

/**

Define a property.
The type of a property is automatically deduced from its default value, and a typedef \c \<name>_type is defined.
The value of a property can be read or written with the \c \<name>() method.

\param name The name of the property.
\param default A default value.
\param meta Meta-data useful for UI generation, or 0 if this property should not appear in the UI.

\pre The class must contain #V_ENABLE_PROPERTIES.

\post The class contains the following new members: \code

  typedef <name>_type; // type of <name>
  const <name>_type &<name>(void) const; // constant getter
  <name>_type &<name>(void); // mutable getter. Use it only if you don't want to be notified when the value changes.
  this_type &<name>(const <name>_type &); // setter
  void load_<name>(InputArchive &); // deserialization
  void save_<name>(OutputArchive &) const; // serialization

\endcode

*/
#define V_DEFINE_PROPERTY(name, default, meta)\
\
  typedef typename v::raw_type<V_DECLTYPE((default))>::type name##_type;\
\
  struct name##_type_\
  {\
    name##_type value;\
\
    name##_type_(void)\
      : value(default)\
    {\
      static const bool registered = properties().insert(make_pair(std::string(#name), v::Property<this_type>(&this_type::load_##name, &this_type::save_##name, meta))).second;\
      (void) registered;\
    }\
  } name##_;\
\
  name##_type &name(void)\
  {\
    return name##_.value;\
  }\
\
  const name##_type &name(void) const\
  {\
    return name##_.value;\
  }\
\
  this_type &name(const name##_type &value)\
  {\
    name##_.value = value;\
    return *this;\
  }\
\
  void load_##name(v::InputArchive &archive)\
  {\
    archive(#name, name##_.value);\
  }\
\
  void save_##name(v::OutputArchive &archive) const\
  {\
    archive(#name, name##_.value);\
  }\

/// A property of a class.
/// This class holds the meta-data necessary for introspection.
template<class T>
struct Property
{
  /// Initialize a new property.
  Property(void (T::*load)(InputArchive &), void (T::*save)(OutputArchive &) const, const char *meta)
    : load(load)
    , save(save)
    , meta(meta)
  {
  }

  /// The function responsible for loading this property.
  void (T::*load)(InputArchive &);

  /// The function responsible for saving this property.
  void (T::*save)(OutputArchive &) const;

  /// Additional meta-data useful for automatic UI generation.
  const char *meta;
};

/// \}
}}

#ifdef DOXYGEN

#undef V_ENABLE_PROPERTIES
#undef V_DEFINE_PROPERTY
#define V_ENABLE_PROPERTIES(class)
#define V_DEFINE_PROPERTY(name, default, meta) /** meta */ Q_PROPERTY(auto name); auto name = default;

#endif
#endif
