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

#include <libv/core/miscmath.hpp>
#include <GL/gl.h>

#include "controller_cad.hpp"
#include "viewer_data.hpp"

namespace v {
namespace graphic {
namespace viewers_ {

ControllerCAD::ControllerCAD()
  : t(0, 0, 0)
  , r(0, 0)
  , s(1)
{
}

QMatrix4x4 ControllerCAD::rotation() const
{
  QMatrix4x4 m;
  m.rotate(r.x() * 180 / M_PI, 1, 0, 0);
  m.rotate(r.y() * 180 / M_PI, 0, 1, 0);
  return m;
}

void ControllerCAD::apply(const ViewerData *that) const
{
  const QSize w = that->that->viewport()->size();

  QMatrix4x4 m;
  m.scale(2. / w.width(), 2. / -w.height(), 1e-5);
  m.rotate(r.x() * 180 / M_PI, 1, 0, 0);
  m.rotate(r.y() * 180 / M_PI, 0, 1, 0);
  m.scale(s);
  m.translate(t);

  glLoadMatrixf(m.data());
}

void ControllerCAD::on_click
( ViewerData *that
, QMouseEvent *event
)
{
  if(that->on_click_)
  {
    const QPointF p = that->that->to_scene(event->pos());
    that->on_click_(float(p.x()), float(p.y()));
  }
}

void ControllerCAD::on_click_right
( ViewerData *that
, QMouseEvent *event
)
{
  that->that->reticle_pos(event->pos());
}

void ControllerCAD::on_drag
( ViewerData *that
, QMouseEvent *event
)
{
  const QPoint d = event->pos() - that->mouse_pos;
  on_scroll(that, d.x(), d.y());
}

void ControllerCAD::on_drag_right
( ViewerData *that
, QMouseEvent *event
)
{
  const QPoint d = event->pos() - that->mouse_pos;
  const QSize w = that->that->viewport()->size();

  r.setX(clamp(r.x() + d.y() * M_PI / w.height(), -M_PI / 2, M_PI / 2));
  r.setY(wrap_pi(r.y() - d.x() * M_PI / w.width()));

  that->update();
}

void ControllerCAD::on_wheel
( ViewerData *that
, QWheelEvent *event
)
{
  const double ds = std::pow(1.0015, event->delta());

  s *= ds;
  t += rotation().transposed() * -QVector3D(QRectF(-event->pos(), that->that->viewport()->size()).center()) * (1 - ds) / s;

  that->update();
}

void ControllerCAD::on_scroll
( ViewerData *that
, int dx
, int dy
)
{
  if(std::abs(r.x()) < .5)
  {
    t += rotation().transposed() * QVector3D(dx, dy, 0) / s;
  }
  else
  {
    const QVector4D a = rotation().row(2) * rotation().row(1).y() / rotation().row(1).z();
    t += (dy * (rotation().row(1) - a) + dx * (rotation().row(0) - a)).toVector3D() / s;
  }

  that->update();
}

}}}
