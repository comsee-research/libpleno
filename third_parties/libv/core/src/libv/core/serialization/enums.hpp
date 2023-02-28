/**

\file
\author Alexis Wilhelm (2014)
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

#ifndef LIBV_CORE_SERIALIZATION_ENUMS_HPP
#define LIBV_CORE_SERIALIZATION_ENUMS_HPP

#include <libv/core/serialization/contiguous_containers.hpp>
#include <boost/format.hpp>

/**

Define a serializable enum.

\param _type The name of the new enum.
\param _macro A macro iterating through the enum values.

\post The following new symbols are defined: \code

  enum _type {...}; // the new enum
  _type to##_type(string); // conversion from string
  string to_string(_type); // conversion to string
  void load(InputArchive, _type); // serialization
  void save(OutputArchive, _type); // serialization

\endcode

A typical usage will look like this: \code

  #define _FOR_EACH_MODE(x) x(PLAYING) x(RECORDING) x(LIVE)
  V_DEFINE_ENUM(Mode, _FOR_EACH_MODE)
  #undef _FOR_EACH_MODE // you might want to undef temporary macros

\endcode

*/
#define V_DEFINE_ENUM(_type, _macro)\
\
  enum _type { _macro(V_DEFINE_ENUM_enumerate_) };\
\
  /** Conversion from string. */\
  inline _type to##_type(const std::string &s)\
  {\
    _macro(V_DEFINE_ENUM_from_string_)\
    throw std::runtime_error(str(boost::format("invalid %s: %s") % #_type % s));\
  }\
\
  /** Serialization. */\
  inline void load(v::InputArchive &archive, _type &x)\
  {\
    std::string s;\
    load(archive, s);\
    x = to##_type(s);\
  }\
\
  /** Conversion to string. */\
  inline std::string to_string(_type x)\
  {\
    switch(x)\
    {\
      _macro(V_DEFINE_ENUM_to_string_)\
      default: throw std::runtime_error(str(boost::format("invalid %s: %s") % #_type % x));\
    }\
  }\
\
  /** Serialization. */\
  inline void save(v::OutputArchive &archive, _type x)\
  {\
    save(archive, to_string(x));\
  }\

#define V_DEFINE_ENUM_enumerate_(x) x,
#define V_DEFINE_ENUM_from_string_(x) if(s == #x) return x;
#define V_DEFINE_ENUM_to_string_(x) case x: return #x;

#endif
