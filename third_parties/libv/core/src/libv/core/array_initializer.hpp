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

#ifndef LIBV_CORE_ARRAY_INITIALIZER_HPP
#define LIBV_CORE_ARRAY_INITIALIZER_HPP

#include "global.hpp"

namespace v {
namespace core {
/// \addtogroup image
/// \{

template<class T, int n>
struct ArrayInitializer
{
  T head;
  ArrayInitializer<T, n - 1> tail;

  ArrayInitializer(const T &head, const ArrayInitializer<T, n - 1> &tail)
    : head(head)
    , tail(tail)
  {
  }

  ArrayInitializer<T, n + 1>
  operator()(const T &new_head) const
  {
    return ArrayInitializer<T, n + 1>(new_head, *this);
  }

  template<class Iterator> void
  apply(Iterator iterator) const
  {
    apply_(iterator);
  }

  template<class Iterator> void
  apply_(Iterator &iterator) const
  {
    tail.apply_(iterator);
    ++iterator;
    *iterator = head;
  }
};

template<class T>
struct ArrayInitializer<T, 1>
{
  T head;

  ArrayInitializer(const T &head)
    : head(head)
  {
  }

  ArrayInitializer<T, 2>
  operator()(const T &new_head) const
  {
    return ArrayInitializer<T, 2>(new_head, *this);
  }

  template<class Iterator> void
  apply(const Iterator &iterator) const
  {
    *iterator = head;
  }

  template<class Iterator> void
  apply_(Iterator &iterator) const
  {
    apply(iterator);
  }
};

template<class T>
struct ArrayInitializer<T, 0>
{
};

/**

Build an array initializer.
Use it to feed initial values to an array, like this: \code
  RGBU8 color = new_array(r)(g)(b);
\endcode

\param first The first element.

\return An array initializer with a single value.
More values are added using \c operator()().

*/
template<class T> inline ArrayInitializer<T, 1>
new_array(const T &first)
{
  return ArrayInitializer<T, 1>(first);
}

/// \}
}}

#endif
