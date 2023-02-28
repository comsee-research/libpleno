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

#include <QPainter>
#include <QScrollBar>
#include <boost/format.hpp>
#include <cmath>

#include "viewer_data.hpp"

namespace v {
namespace graphic {
namespace viewers_ {

static const int
  margin = 10, // distance in pixels between axes and window border
  tick_size = 3; // number of pixels for ticks

static qreal
step(qreal &min, const QTransform &m)
{
  qreal e = std::log10(1000 / m.m11());
  const int n = int(std::floor(e - std::log10(1.5)));
  e = std::pow(10, e - n);
  if(e <= 2) e = 2 * std::pow(10, n - 1);
  else if(e <= 5) e = 5 * std::pow(10, n - 1);
  else e = std::pow(10, n);
  min = std::ceil(min / e) * e;
  return e;
}

static QString
to_string(qreal x)
{
  return " " + QString::fromStdString(str(boost::format("%10g") % x)).trimmed() + " ";
}

ViewerData::ViewerData(Viewer *that)
  : that(that)
  , axes_enabled(false)
  , grid_enabled(false)
  , reticle_enabled(false)
  , background_brush(Qt::white)
  , axes_type(AxesUV)
  , update_guard(false)
  , has_size(false)
  , controllers(that)
{
  axes_pen.setColor(Qt::black);
  axes_pen.setWidth(3);
  grid_pen.setColor(QColor(0, 0, 0, 0x80));
  grid_pen.setWidth(1);
  reticle_pen.setColor(Qt::red);
  reticle_pen.setWidth(1);
}

void ViewerData::update()
{
  if(update_guard) return;
  update_guard = true;

  if(!has_size && !that->bounds().isEmpty())
  {
    that->zoom_original();
  }

  that->viewport()->update();

  if(that->reticle_is_visible())
  {
    that->reticle_pos(that->reticle_pos());
  }

  const QRect contents = that->from_scene(that->bounds());
  QScrollBar *hbar = that->horizontalScrollBar(), *vbar = that->verticalScrollBar();
  that->blockSignals(true);
  hbar->setValue(0);
  vbar->setValue(0);
  that->blockSignals(false);
  hbar->setRange(std::min(0, contents.left()), std::max(0, contents.right() - hbar->pageStep()));
  vbar->setRange(std::min(0, contents.top()), std::max(0, contents.bottom() - vbar->pageStep()));
  that->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  that->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

  update_guard = false;
}

Controller &ViewerData::controller()
{
  Controller *array[] = {
    &controller_2d,
    &controller_cad,
  };

  return *array[controllers.checkedAction()->data().toInt()];
}

void ViewerData::draw_axes_uv
( QPainter &painter
, const QFontMetrics &metrics
)
{
  const QPoint
    o_view(margin, margin),
    x_view(that->viewport()->width() - margin, margin),
    y_view(margin, that->viewport()->height() - margin);
  const QPointF
    x_scene = that->to_scene(x_view),
    y_scene = that->to_scene(y_view);
  qreal
    xmin = y_scene.x(),
    ymin = x_scene.y();
  const qreal
    xmax = x_scene.x(),
    ymax = y_scene.y(),
    dx = step(xmin, transform),
    dy = step(ymin, transform);
  const int
    x_text = y_view.x() + tick_size,
    y_text = x_view.y() + tick_size + metrics.ascent(),
    y_grid = y_text + metrics.descent();

  painter.drawLine(o_view, x_view);
  painter.drawLine(o_view, y_view);
  painter.drawLine(x_view, QPoint(x_view.x() - margin, x_view.y() - margin));
  painter.drawLine(x_view, QPoint(x_view.x() - margin, x_view.y() + margin));
  painter.drawLine(y_view, QPoint(y_view.x() - margin, y_view.y() - margin));
  painter.drawLine(y_view, QPoint(y_view.x() + margin, y_view.y() - margin));

  for(qreal x = xmin; x < xmax; x += dx)
  {
    static const QPoint dp(0, tick_size);
    const QPoint p(that->from_scene(x, 0).x(), x_view.y());
    painter.drawLine(p - dp, p + dp);
    const QString text = to_string(x);
    painter.drawText(p.x() - metrics.width(text) / 2, y_text, text);

    if(grid_enabled)
    {
      painter.save();
      painter.setPen(grid_pen);
      painter.drawLine(p.x(), y_grid, p.x(), y_view.y());
      painter.restore();
    }
  }

  for(qreal y = ymin; y < ymax; y += dy)
  {
    static const QPoint dp(tick_size, 0);
    const QPoint p(y_view.x(), that->from_scene(0, y).y());
    painter.drawLine(p - dp, p + dp);
    const QString text = to_string(y);
    painter.drawText(x_text, p.y() + metrics.xHeight() / 2, text);

    if(grid_enabled)
    {
      painter.save();
      painter.setPen(grid_pen);
      painter.drawLine(x_text + metrics.width(text), p.y(), x_view.x(), p.y());
      painter.restore();
    }
  }
}

void ViewerData::draw_axes_xy
( QPainter &painter
, const QFontMetrics &metrics
)
{
  const QPoint
    o_view(margin, that->viewport()->height() - margin),
    x_view(that->viewport()->width() - margin, that->viewport()->height() - margin),
    y_view(margin, margin);
  const QPointF
    x_scene = that->to_scene(x_view),
    y_scene = that->to_scene(y_view);
  qreal
    xmin = y_scene.x(),
    ymin = y_scene.y();
  const qreal
    xmax = x_scene.x(),
    ymax = x_scene.y(),
    dx = step(xmin, transform),
    dy = step(ymin, transform);
  const int
    x_text = y_view.x() + tick_size,
    y_text = x_view.y() - tick_size - metrics.descent(),
    y_grid = y_text - metrics.ascent();

  painter.drawLine(o_view, x_view);
  painter.drawLine(o_view, y_view);
  painter.drawLine(x_view, QPoint(x_view.x() - margin, x_view.y() - margin));
  painter.drawLine(x_view, QPoint(x_view.x() - margin, x_view.y() + margin));
  painter.drawLine(y_view, QPoint(y_view.x() - margin, y_view.y() + margin));
  painter.drawLine(y_view, QPoint(y_view.x() + margin, y_view.y() + margin));

  for(qreal x = xmin; x < xmax; x += dx)
  {
    static const QPoint dp(0, tick_size);
    const QPoint p(that->from_scene(x, 0).x(), x_view.y());
    painter.drawLine(p - dp, p + dp);
    const QString text = to_string(x);
    painter.drawText(p.x() - metrics.width(text) / 2, y_text, text);

    if(grid_enabled)
    {
      painter.save();
      painter.setPen(grid_pen);
      painter.drawLine(p.x(), y_grid, p.x(), y_view.y());
      painter.restore();
    }
  }

  for(qreal y = ymin; y < ymax; y += dy)
  {
    static const QPoint dp(tick_size, 0);
    const QPoint p(y_view.x(), that->from_scene(0, y).y());
    painter.drawLine(p - dp, p + dp);
    const QString text = to_string(-y);
    painter.drawText(x_text, p.y() + metrics.xHeight() / 2, text);

    if(grid_enabled)
    {
      painter.save();
      painter.setPen(grid_pen);
      painter.drawLine(x_text + metrics.width(text), p.y(), x_view.x(), p.y());
      painter.restore();
    }
  }
}

}}}
