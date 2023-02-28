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
#include <GL/gl.h>

#include "controller_2d.hpp"
#include "viewer_data.hpp"

namespace v {
namespace graphic {
namespace viewers_ {

void Controller2D::apply(const ViewerData *that) const
{
  const QTransform &m = that->transform;
  const QSize w = that->that->viewport()->size();

  const double transform[] = {
    m.m11(), m.m12(), 0, m.m13(),
    m.m21(), m.m22(), 0, m.m23(),
    0, 0, 0, 0,
    m.m31(), m.m32(), 0, m.m33(),
  };

  glLoadIdentity();
  glTranslated(-1, 1, 0);
  glScaled(2. / w.width(), 2. / -w.height(), 0);
  glMultMatrixd(transform);
}

void Controller2D::on_click
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

void Controller2D::on_click_right
( ViewerData *that
, QMouseEvent *event
)
{
  that->that->reticle_pos(event->pos());
}

void Controller2D::on_drag
( ViewerData *that
, QMouseEvent *event
)
{
  const QPoint d = event->pos() - that->mouse_pos;
  on_scroll(that, d.x(), d.y());
}

void Controller2D::on_wheel
( ViewerData *that
, QWheelEvent *event
)
{
  const qreal s = std::pow(1.0015, event->delta());
  that->transform *= QTransform::fromTranslate(-event->x(), -event->y());
  that->transform *= QTransform::fromScale(s, s);
  that->transform *= QTransform::fromTranslate(event->x(), event->y());
  that->update();
}

void Controller2D::on_scroll
( ViewerData *that
, int dx
, int dy
)
{
  if(that->update_guard) return;
  that->transform *= QTransform::fromTranslate(dx, dy);
  that->update();
}

}}}
