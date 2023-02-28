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

#ifndef LIBV_CORE_TYPE_TRAITS_HAS_TYPE_HPP
#define LIBV_CORE_TYPE_TRAITS_HAS_TYPE_HPP

#include "../global.hpp"

namespace v {
namespace core {
/// \addtogroup type_traits
/// \{

/// Define a boolean type trait that is \c true if its parameter contains a typedef \e name.
#define V_DEFINE_HAS_TYPE(name)\
\
  /** Check if the type \e T contains a \c typedef \c name. */\
  template<class T>\
  class has_##name\
  {\
    typedef int yes[1];\
    typedef int no[2];\
    template<class X> static yes &test(typename X::name *);\
    template<class X> static no &test(...);\
  public:\
    /** Result */\
    enum {value = sizeof(test<T>(0)) == sizeof(yes)};\
  };\

V_DEFINE_HAS_TYPE(container_type)

/// \}
}}

#endif
