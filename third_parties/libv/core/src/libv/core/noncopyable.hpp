/**

\file
\author Stéphane Witzmann (2014)
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

#ifndef LIBV_CORE_NONCOPYABLE_HPP
#define LIBV_CORE_NONCOPYABLE_HPP

#include <libv/core/global.hpp>

namespace v {
namespace core {

/**
 * Base class for everything that should not be copied,
 * either via operator= or copy/move constructor.
 */
struct LIBV_CORE_EXPORT noncopyable
{
  constexpr noncopyable() {}

  noncopyable(const noncopyable &) = delete;
  noncopyable(noncopyable &&) = delete;
  noncopyable & operator=(const noncopyable &) = delete;
};

/**
* Same as noncopyable with a virtual destructor.
* For use with classes that may be overloaded by the user.
*/
struct LIBV_CORE_EXPORT virtual_noncopyable: public noncopyable
{
  constexpr virtual_noncopyable() {}
  virtual ~virtual_noncopyable();
};

}
}

#endif
