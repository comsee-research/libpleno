/**

\file
ViewerContext class definition.
This header should be included by the user.
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

#ifndef LIBV_GRAPHIC_VIEWER_CONTEXT_HPP
#define LIBV_GRAPHIC_VIEWER_CONTEXT_HPP

#include <string>
#include <functional>

#include "colors.hpp"
#include "types.hpp"

namespace v {
namespace graphic {
/// \addtogroup viewers
/// \{

/// A handle for the viewer system.
struct LIBV_GRAPHIC_EXPORT ViewerContext
{
  /**

  \name Functions affecting the current viewer.
  Those commands are sent immediately to the viewer system.

  \{

  */
  /**

  Commit all pending commands for the current layer.

  */
  ViewerContext &update(void);
  /**

  Show or hide the current layer.

  */
  ViewerContext &show(bool);
  /**

  Set the current layer's name.

  */
  ViewerContext &name(const std::string &);
  /**

  Set the interaction mode.

  */
  ViewerContext &interaction_mode(InteractionMode);
  /**

  Set the axes convention.

  */
  ViewerContext &axes(ViewerAxesType);
  /**

  Set the background color.

  */
  ViewerContext &background(const v::RGBAU8 &color);
  /**

  Resize this viewer.

  */
  ViewerContext &size(int width, int height);
  /**

  Set the window title.

  */
  ViewerContext &title(const std::string &);
  /**

  Register a function that will be called for each click event.

  */
  ViewerContext &on_click(const std::function<void(float, float)> &);
  /**

  Center the view on a point.

  */
  ViewerContext &look_at(float x, float y);
  /**

  Set the zoom factor.

  */
  ViewerContext &zoom(float zoom);
  /**

  \}

  \name Functions affecting the current layer.
  Those commands are sent to the viewer system when you call update().

  \{

  */
  /**

  Clear the current layer.

  \returns A reference to \e this.

  \note About the \c clear_ variable:
  If we clear the command list here, the viewer might never show anything (if the display is updated after the clear() command but before the update() command).
  To prevent this we let the update() function clear the command list if the clear() command has been called at least once.

  */
  ViewerContext &clear(void);
  /**

  Draw a point.

  */
  ViewerContext &add_point(float x, float y);
  /**

  Draw a line.

  */
  ViewerContext &add_line(float x1, float y1, float x2, float y2);
  /**

  Draw an arrow.

  */
  ViewerContext &add_arrow(float x_tail, float y_tail, float x_head, float y_head, bool both_sides = false);
  /**

  Draw an axis-aligned rectangle.

  */
  ViewerContext &add_rect(float left, float top, float width, float height);
  /**

  Draw a triangle.

  */
  ViewerContext &add_triangle(float x1, float y1, float x2, float y2, float x3, float y3);
  /**

  Draw a quadrilateral.

  */
  ViewerContext &add_quad(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4);
  /**

  Draw a circle.

  */
  ViewerContext &add_circle(float x_center, float y_center, float radius);
  /**

  Draw an ellipse.

  */
  ViewerContext &add_ellipse(float x_center, float y_center, float x_radius, float y_radius);
  /**

  Paste an image.

  */
  ViewerContext &add_image(float left, float top, const v::ImageU8cr &image);
  /**

  Paste an image.

  */
  ViewerContext &add_image(float left, float top, const v::ImageRGBU8cr &image);
  /**

  Paste an image.

  */
  ViewerContext &add_image(float left, float top, const v::ImageRGBAU8cr &image);
  /**

  Draw a text.

  */
  ViewerContext &add_text(float x, float y, const std::string &text);
  /**

  Call a function.

  */
  ViewerContext &add_opengl(const std::function<void()> &);
  /**

  \}

  */
  ViewerContext &viewer(int);
  ViewerContext &layer(int);
  ViewerContext &pen_width(int);
  ViewerContext &pen_color(const v::RGBAU8 &);
  ViewerContext &pen_style(PenStyle);
  ViewerContext &brush_color(const v::RGBAU8 &);
  ViewerContext &brush_style(BrushStyle);
  ViewerContext &point_style(PointStyle);
  ViewerContext &font_family(FontFamily);
  ViewerContext &font_style(FontStyle);
  ViewerContext &font_direction(FontDirection);
  ViewerContext &font_size(int);

  ViewerContext(void);
  int viewer(void) const;
  int layer(void) const;
private:
  friend struct Viewer;
  friend union viewers_::Commands;
  int viewer_, layer_;
  uint32_t brush_color_;
  BrushStyle brush_style_;
  FontDirection font_direction_;
  FontFamily font_family_;
  int font_size_;
  FontStyle font_style_;
  uint32_t pen_color_;
  PenStyle pen_style_;
  int pen_width_;
  PointStyle point_style_;
};

/// \}
}}

#endif
