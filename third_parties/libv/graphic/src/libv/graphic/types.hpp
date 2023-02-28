/**

\file
\author Laurent Lequievre (2012)
\copyright 2012 Institut Pascal

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

#ifndef LIBV_GRAPHIC_TYPES_HPP
#define LIBV_GRAPHIC_TYPES_HPP

#include "global.hpp"

namespace v {
namespace graphic {
/// \addtogroup viewers
/// \{

/// An axes convention.
enum ViewerAxesType
{
  AxesXY, ///< The origin is at the bottom left corner.
  AxesUV ///< The origin is at the top left corner.
};

/// A text orientation.
enum FontDirection
{
  Vertical, ///< Top to bottom.
  Horizontal ///< Left to right.
};

/// A font family.
enum FontFamily
{
  Times,
  Arial,
  Courier
};

/// A font style.
enum FontStyle
{
  Normal,
  Italic,
  Bold
};

/// A point style.
enum PointStyle
{
  Pixel = 0, ///< Square.
  Round = 1, ///< Circle.
  Cross = 2 ///< Plus symbol.
};

/// A pen style.
enum PenStyle
{
  NoPen = 0, ///< No line at all.
  SolidLine = 1, ///< A plain line.
  DashLine = 2, ///< Dashes separated by a few pixels.
  DotLine = 3, ///< Dots separated by a few pixels.
  DashDotLine = 4, ///< Alternate dots and dashes.
  DashDotDotLine = 5 ///< One dash, two dots, one dash, two dots.
};

/// A brush style.
enum BrushStyle
{
  NoBrush = 0,
  SolidPattern = 1,
  Dense1Pattern = 2,
  Dense2Pattern = 3,
  Dense3Pattern = 4,
  Dense4Pattern = 5,
  Dense5Pattern = 6,
  Dense6Pattern = 7,
  Dense7Pattern = 8,
  HorPattern = 9,
  VerPattern = 10,
  CrossPattern = 11,
  BDiagPattern = 12,
  FDiagPattern = 13,
  DiagCrossPattern = 14
};

/**

How the mouse/keyboard events affect the camera.

*/
enum InteractionMode
{
  /**

  The camera always is always parallel to the XY plane.
  Dragging with the left button translates the camera.
  The mouse wheel handles the zoom.

  */
  INTERACTION_2D,
  /**

  The camera is fixed to a lever which turns around a pivot.
  Dragging with the left button translates the pivot.
  Dragging with the right button rotates the lever around the pivot.
  The mouse wheel handles the zoom.

  */
  INTERACTION_CAD,
  /**

  The camera navigates in the scene.
  Pressing the left button rotates the camera towards the mouse pointer.
  Pressing the keys [↑], [↓], [←], [→] translate the camera.

  */
  INTERACTION_FPS
};

/// \}
}}

#endif
