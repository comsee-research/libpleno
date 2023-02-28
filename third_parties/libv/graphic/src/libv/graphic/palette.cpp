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

#include <cmath>
#include "palette.hpp"

namespace v {
namespace graphic {
namespace palette_ {

static bool
acceptable(std::vector<Item> &cache, float hue)
{
  for(Item &color: cache)
  {
    if(color.hue == hue)
    {
      return false;
    }
  }
  return true;
}

static v::RGBAU8
make_color(float hue)
{
  const uint8_t n = uint8_t((hue - float(int(hue))) * 255);
  const uint8_t m = uint8_t(255 - n);
  switch(int(hue))
  {
    case 0: return v::new_array<uint8_t>(255)(n)(0)(255);
    case 1: return v::new_array<uint8_t>(m)(255)(0)(255);
    case 2: return v::new_array<uint8_t>(0)(255)(n)(255);
    case 3: return v::new_array<uint8_t>(0)(m)(255)(255);
    case 4: return v::new_array<uint8_t>(n)(0)(255)(255);
    case 5: return v::new_array<uint8_t>(255)(0)(m)(255);
    default: return v::new_array<uint8_t>(0)(0)(0)(0);
  }
}

LIBV_GRAPHIC_EXPORT void
grow(std::vector<Item> &cache)
{
  float hue = cache.empty() ? 0 : cache.back().hue;
  for(float step = 3; !acceptable(cache, hue); step /= -2)
  {
    hue = std::fmod(hue + step + 6, 6.f);
  }
  const Item color = {true, false, hue, make_color(hue)};
  cache.push_back(color);
}

LIBV_GRAPHIC_EXPORT void
clean(std::vector<Item> &cache)
{
  for(Item &color: cache)
  {
    color.free = !color.allocated;
    color.allocated = false;
  }
}

}}}
