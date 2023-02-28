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

#ifndef LIBV_GRAPHIC_PRIVATE_CONTROLLER_HPP
#define LIBV_GRAPHIC_PRIVATE_CONTROLLER_HPP

#include <QMatrix4x4>
#include <QMouseEvent>

namespace v {
namespace graphic {
namespace viewers_ {

struct ViewerData;

struct Controller
{
  virtual void on_click(ViewerData *, QMouseEvent *) = 0;
  virtual void on_click_right(ViewerData *w, QMouseEvent *e) { on_click(w, e); }
  virtual void on_drag(ViewerData *, QMouseEvent *) = 0;
  virtual void on_drag_right(ViewerData *w, QMouseEvent *e) { on_drag(w, e); }
  virtual void on_wheel(ViewerData *, QWheelEvent *) = 0;
  virtual void on_scroll(ViewerData *, int, int) = 0;
  virtual void apply(const ViewerData *) const = 0;
};

}}}

#endif
