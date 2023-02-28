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

#ifndef LIBV_GRAPHIC_PRIVATE_STYLE_HPP
#define LIBV_GRAPHIC_PRIVATE_STYLE_HPP

#include <QPen>

#include "../viewer.hpp"

namespace v {
namespace graphic {

/// Some style information used to draw the scene.
struct Viewer::Style
{
  /// \privatesection
  Style(const ViewerContext &);
  bool operator<(const Style &) const;
  QPen pen;
  QBrush brush;
  QFont font;
  PointStyle point_style;
  FontDirection font_direction;
};

}}

#endif
