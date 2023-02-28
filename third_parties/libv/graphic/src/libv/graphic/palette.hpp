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

#ifndef LIBV_GRAPHIC_PALETTE_HPP
#define LIBV_GRAPHIC_PALETTE_HPP

#include <libv/core/image/image.hpp>
#include <vector>

#include "global.hpp"

namespace v {
namespace graphic {
namespace palette_ {
/// \privatesection

struct Item
{
  bool allocated, free;
  float hue;
  v::RGBAU8 color;
};

LIBV_GRAPHIC_EXPORT void grow(std::vector<Item> &);
LIBV_GRAPHIC_EXPORT void clean(std::vector<Item> &);

}
/// \addtogroup viewers
/// \{

/**

Generate stable, unique colors for objects.

A typical usage will look like this: \code

  Palette<int> palette;

  while(something_to_do)
  {
    for(auto object: objects)
    {
      RGBAU8 color = palette[object.id()];
      ... // draw some shapes with this color
    }
    palette.clean();
  }

\endcode

*/
template<class T>
class Palette
{
  std::vector<T> cached_objects_;
  std::vector<palette_::Item> cached_colors_;

public:

  /**

  Get the color of the given object.

  \param x An object.
  \return A color.

  The algorithm to find the color of the given object is as follow:

  */
  v::RGBAU8
  operator[](const T &x)
  {
    /// <ol>

    /// <li> If we already have a color for this object, we just return this color.
    for(size_t i = 0; i < cached_colors_.size(); ++i)
    {
      if(cached_objects_[i] == x)
      {
        cached_colors_[i].allocated = true;
        return cached_colors_[i].color;
      }
    }

    /// <li> If we have unused colors, we associate an unused color to this object.
    for(size_t i = 0; i < cached_colors_.size(); ++i)
    {
      if(cached_colors_[i].free && !cached_colors_[i].allocated)
      {
        cached_objects_[i] = x;
        cached_colors_[i].allocated = true;
        return cached_colors_[i].color;
      }
    }

    /// <li> If we have no more free slots, we need to generate a new color.
    palette_::grow(cached_colors_);
    cached_objects_.push_back(x);
    return cached_colors_.back().color;

    /// </ol>
  }

  /**

  Forget old objects to make room for new ones.

  */
  void
  clean(void)
  {
    palette_::clean(cached_colors_);
  }
};

/// \}
}}

#endif
