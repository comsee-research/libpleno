/**

\file
\author Vadim Litvinov (2013)
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

#ifndef LIBV_CORE_DRAW_TOOLS_HPP
#define LIBV_CORE_DRAW_TOOLS_HPP

#include <cmath>
#include <algorithm>
#include "miscmath.hpp"

namespace libv {
namespace core {
  /// \addtogroup image
  /// \{

  /// \brief Fill the image with the given color
  ///
  /// Uniformly fills the image with the given color.
  ///
  /// \param  image Image to fill.
  /// \param  color Filling color.
  template<class Image>
  inline void draw_fill(Image& image, typename Image::const_reference_to_pixel color) {
    std::fill(image.begin_pixel(), image.end_pixel(), color);
  }

  /// \brief Draw a point
  ///
  /// Draw a point on the given image.
  ///
  /// \param  image Image to draw to.
  /// \param  x     Point horizontal position.
  /// \param  y     Point vertical position.
  /// \param  color Point color.
  template<class Image>
  inline void draw_point(Image& image, float x, float y, typename Image::const_reference_to_pixel color)
  {
    image(round(y), round(x)) = color;
  }

  /// \brief Draw line
  ///
  /// Draw a line on the given image.
  ///
  /// \param  image Image to draw to.
  /// \param  x1    First point horizontal position.
  /// \param  y1    First point vertical position.
  /// \param  x2    Second point horizontal position.
  /// \param  y2    Second point vertical position.
  /// \param  color Line color.
  template<class Image>
  inline void draw_line(Image& image, float x1, float y1, float x2, float y2, typename Image::const_reference_to_pixel color)
  {
    const bool steep = std::fabs(y2 - y1) > std::fabs(x2 - x1);
    if(steep)
    {
      std::swap(x1, y1);
      std::swap(x2, y2);
    }

    if(x1 > x2)
    {
      std::swap(x1, x2);
      std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = std::fabs(y2 - y1);

    float error = dx/2.0f;
    const int ystep = (y1 < y2)?1:-1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for(int x = (int)x1; x < maxX; ++x)
    {
      if(steep) image(x, y) = color;
      else image(y, x) = color;

      error -= dy;
      if(error < 0)
      {
        y += ystep;
        error += dx;
      }
    }
  }

  /// \brief Draw rectangle
  ///
  /// Draw a rectangle outline on the given image.
  ///
  /// \param  image Image to draw to.
  /// \param  x1    First corner horizontal position.
  /// \param  y1    First corner vertical position.
  /// \param  x2    Second corner horizontal position.
  /// \param  y2    Second corner vertical position.
  /// \param  color Rectangle color.
  template<class Image>
  inline void draw_rect(Image& image, float x1, float y1, float x2, float y2, typename Image::const_reference_to_pixel color)
  {
    draw_line(image, x1, y1, x2, y1, color);
    draw_line(image, x1, y1, x1, y2, color);
    draw_line(image, x2, y1, x2, y2, color);
    draw_line(image, x1, y2, x2, y2, color);
  }

  /// \brief Draw circle
  ///
  /// Draw a circle on the given image.
  ///
  /// \param  image   Image to draw to.
  /// \param  cx      Circle center horizontal position.
  /// \param  cy      Circle center vertical position.
  /// \param  radius  Circle radius.
  /// \param  color   Circle color.
  template<class Image>
  inline void draw_circle(Image& image, float cx, float cy, float radius, typename Image::const_reference_to_pixel color) {
    const int x0 = (int)round(cx);
    const int y0 = (int)round(cy);
    const int r = (int)round(radius);

    int x = r, y = 0;
    int x_chg = 1 - (r << 1);
    int y_chg = 0;
    int r_err = 0;

    while(x >= y)
    {
      image(y + y0, x + x0) = color;
      image(x + y0, y + x0) = color;
      image(y + y0, -x + x0) = color;
      image(x + y0, -y + x0) = color;
      image(-y + y0, -x + x0) = color;
      image(-x + y0, -y + x0) = color;
      image(-y + y0, x + x0) = color;
      image(-x + y0, y + x0) = color;

      ++y;
      r_err += y_chg;
      y_chg += 2;
      if((r_err << 1) + x_chg > 0)
      {
        --x;
        r_err += x_chg;
        x_chg += 2;
      }
    }
  }

  /// \}
}}

#endif
