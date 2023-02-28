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

#ifndef LIBV_GRAPHIC_PRIVATE_CONTROLLER_CAD_HPP
#define LIBV_GRAPHIC_PRIVATE_CONTROLLER_CAD_HPP

#include <QVector2D>

#include "controller.hpp"

namespace v {
namespace graphic {
namespace viewers_ {

struct ControllerCAD: Controller
{
  ControllerCAD();
  void on_click(ViewerData *, QMouseEvent *);
  void on_click_right(ViewerData *, QMouseEvent *);
  void on_drag(ViewerData *, QMouseEvent *);
  void on_drag_right(ViewerData *, QMouseEvent *);
  void on_wheel(ViewerData *, QWheelEvent *);
  void on_scroll(ViewerData *, int, int);
  void apply(const ViewerData *) const;
  QMatrix4x4 rotation() const;
  QVector3D t;
  QVector2D r;
  double s;
};

}}}

#endif
