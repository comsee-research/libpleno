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

#include "../viewer_context.hpp"
#include "style.hpp"

namespace v {
namespace graphic {

Viewer::Style::Style(const ViewerContext &context)
  : point_style(context.point_style_)
  , font_direction(context.font_direction_)
{
  pen.setStyle(Qt::PenStyle(context.pen_style_));
  pen.setColor(QColor::fromRgba(context.pen_color_));
  pen.setWidth(context.pen_width_);
  pen.setCapStyle(point_style == Round ? Qt::RoundCap : Qt::SquareCap);
  pen.setCosmetic(true);

  brush.setColor(QColor::fromRgba(context.brush_color_));
  brush.setStyle(Qt::BrushStyle(context.brush_style_));

  font.setPixelSize(context.font_size_);

  switch(context.font_style_)
  {
    case Normal: break;
    case Bold: font.setBold(true); break;
    case Italic: font.setItalic(true); break;
  }
  switch(context.font_family_)
  {
    case Arial: font.setFamily("Arial"); break;
    case Times: font.setFamily("Times"); break;
    case Courier: font.setFamily("Courier"); break;
  }
}

bool
Viewer::Style::operator<(const Style &other) const
{
  const char *b1 = reinterpret_cast<const char *>(this), *b2 = reinterpret_cast<const char *>(&other), *e1 = b1 + sizeof(Style), *e2 = b2 + sizeof(Style);
  return std::lexicographical_compare(b1, e1, b2, e2);
}

}}
