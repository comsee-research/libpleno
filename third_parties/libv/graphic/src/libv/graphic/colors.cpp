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

#include <libv/core/image/image.hpp>

#include "colors.hpp"

namespace v {
namespace graphic {

#ifdef DOXYGEN
  /// Define a color.
  /// Also, get around the most vexing parse ambiguity in Doxygen.
  #define _DEFINE_COLOR(_name, r, g, b, a)\
    /** \htmlonly <span style="color:rgb(r, g, b)"> ████ </span> \endhtmlonly */\
    const v::RGBAU8 _name = {r, g, b, a};
#else
  #define _DEFINE_COLOR(_name, r, g, b, a)\
    const RGBAU8 _name = new_array<uint8_t>(r)(g)(b)(a);
#endif

_DEFINE_COLOR(black, 0, 0, 0, 255)
_DEFINE_COLOR(white, 255, 255, 255, 255)
_DEFINE_COLOR(gray, 127, 127, 127, 255)
_DEFINE_COLOR(light_gray, 191, 191, 191, 255)
_DEFINE_COLOR(dark_gray, 63, 63, 63, 255)

_DEFINE_COLOR(red, 255, 0, 0, 255)
_DEFINE_COLOR(light_red, 255, 127, 127, 255)
_DEFINE_COLOR(dark_red, 127, 0, 0, 255)
_DEFINE_COLOR(yellow, 255, 255, 0, 255)
_DEFINE_COLOR(light_yellow, 255, 255, 127, 255)
_DEFINE_COLOR(dark_yellow, 127, 127, 0, 255)
_DEFINE_COLOR(green, 0, 255, 0, 255)
_DEFINE_COLOR(light_green, 127, 255, 127, 255)
_DEFINE_COLOR(dark_green, 0, 127, 0, 255)
_DEFINE_COLOR(cyan, 0, 255, 255, 255)
_DEFINE_COLOR(light_cyan, 127, 255, 255, 255)
_DEFINE_COLOR(dark_cyan, 0, 127, 127, 255)
_DEFINE_COLOR(blue, 0, 0, 255, 255)
_DEFINE_COLOR(light_blue, 127, 127, 255, 255)
_DEFINE_COLOR(dark_blue, 0, 0, 127, 255)
_DEFINE_COLOR(magenta, 255, 0, 255, 255)
_DEFINE_COLOR(light_magenta, 255, 127, 255, 255)
_DEFINE_COLOR(dark_magenta, 127, 0, 127, 255)

_DEFINE_COLOR(orange, 255, 127, 0, 255)
_DEFINE_COLOR(pink, 255, 0, 127, 255)
_DEFINE_COLOR(chartreuse, 127, 255, 0, 255)
_DEFINE_COLOR(spring, 0, 255, 127, 255)
_DEFINE_COLOR(purple, 127, 0, 255, 255)
_DEFINE_COLOR(azure, 0, 127, 255, 255)

}}
