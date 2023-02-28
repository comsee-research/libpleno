/**

\file
Viewer class definition.
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

#include <QApplication>
#include <QFileDialog>
#include <QFont>
#include <QGLWidget>
#include <QMenu>
#include <QPrinter>
#include <QScrollBar>
#include <QSvgGenerator>
#include <QToolButton>
#include <cmath>

#include "private/global.hpp"
#include "private/viewer_data.hpp"

namespace v {
namespace graphic {
namespace {

using namespace viewers_;

static void init_menu(QToolBar *toolbar, QAction *action)
{
  action->setMenu(new QMenu);
  static_cast<QToolButton *>(toolbar->widgetForAction(action))->setPopupMode(QToolButton::InstantPopup);
}

}

/// Initialize this viewer.
Viewer::Viewer(QWidget *parent)
  : QAbstractScrollArea(parent)
  , that(new ViewerData(this))
{
  that->setupUi(this);
  that->toolbar->addWidget(that->coordinates);
  that->dock->setTitleBarWidget(that->titlebar);
  that->dock->setParent(0);
  that->window->setParent(0);

  setViewport(that->viewport);

  init_menu(that->toolbar, that->menu_visibility);
  init_menu(that->toolbar, that->menu_options);

  QAction *actions[] = {
    that->action_interaction_mode_2d,
    that->action_interaction_mode_cad,
    that->action_interaction_mode_fps,
  };

  for(size_t i = 0; i < 3; ++i)
  {
    menu()->addAction(actions[i]);
    actions[i]->setActionGroup(&that->controllers);
    actions[i]->setData(QVariant::fromValue(i));
    actions[i]->setChecked(!i);
  }
}

Viewer::~Viewer(void)
{
  delete that;
}

/**

Put this viewer in a QMainWindow.

\return  A window containing this viewer.

*/
QMainWindow *Viewer::as_window()
{
  that->window->setCentralWidget(this);
  that->window->addToolBar(that->toolbar);
  that->window->show();
  return that->window;
}

/**

Put this viewer in a QDockWidget.

\return  A dock containing this viewer.

*/
QDockWidget *Viewer::as_dock()
{
  that->dock->setWidget(this);
  that->dock_layout->insertWidget(1, that->toolbar);
  that->window->hide();
  return that->dock;
}

/**

The main menu for this viewer.

\return A menu.

*/
QMenu *Viewer::menu()
{
  return that->menu_options->menu();
}

/// The visibility of the axes.
void
Viewer::axes_are_visible(bool value)
{
  if(!value)
  {
    that->action_show_grid->setChecked(false);
  }
  that->axes_enabled = value;
  that->update();
}

/// The visibility of the grid.
void
Viewer::grid_is_visible(bool value)
{
  if(value)
  {
    that->action_show_axes->setChecked(true);
  }
  that->grid_enabled = value;
  that->update();
}

/// The visibility of the reticle.
bool
Viewer::reticle_is_visible(void) const
{
  return that->action_show_reticle->isChecked();
}

/// The visibility of the reticle.
void
Viewer::reticle_is_visible(bool value)
{
  if(value)
  {
    reticle_pos(QPoint(viewport()->width(), viewport()->height()) / 2);
  }
  else
  {
    that->coordinates->setText(QString());
  }
  that->reticle_enabled = value;
  that->update();
}

/// The position of the reticle.
QPoint
Viewer::reticle_pos(void) const
{
  return that->reticle_pos;
}

/// The position of the reticle.
void
Viewer::reticle_pos(const QPoint &new_pos)
{
  that->reticle_pos = new_pos;
  QPointF p = to_scene(new_pos);
  that->coordinates->setText(QString("%1, %2").arg(p.x()).arg(that->axes_type == AxesXY ? -p.y() : p.y()));
  that->update();
}

/// Adjust the scene so that it fits in the viewport.
void
Viewer::zoom_fit_best(void)
{
  const QRectF rect = bounds();
  const qreal s = std::min(width() / rect.width(), height() / rect.height());
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  that->transform.reset();
  that->transform *= QTransform::fromTranslate(-rect.x(), -rect.y());
  that->transform *= QTransform::fromScale(s, s);
  that->update();
}

/// Adjust the viewport so that the scene fits in it.
void
Viewer::zoom_original(void)
{
  const QRectF rect = bounds();
  setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  that->transform.reset();
  that->transform *= QTransform::fromTranslate(-rect.x(), -rect.y());
  window()->resize(window()->size() - size() + rect.size().toSize());
  that->has_size = true;
  that->update();
}

void Viewer::zoom(float zoom)
{
  that->transform *= QTransform::fromTranslate(-width() / 2, -height() / 2);
  that->transform *= QTransform::fromScale(zoom / that->transform.m11(), zoom / that->transform.m11());
  that->transform *= QTransform::fromTranslate(width() / 2, height() / 2);
  that->update();
}

void Viewer::look_at(QPointF point)
{
  QPointF d = QPointF(width(), height()) / 2 - that->transform.map(point);
  that->transform *= QTransform::fromTranslate(d.x(), d.y());
  that->update();
}

/// Save the current state of this viewer as an image.
void
Viewer::save_screenshot(void)
{
  QString file = QFileDialog::getSaveFileName(this, "Save Screen", "", "Images (*.jpg *.png *.pdf *.svg)");
  QString suffix = QFileInfo(file).suffix().toLower();

  if(suffix == "pdf")
  {
    QPrinter printer;
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setOutputFileName(file);
    printer.setPaperSize(size(), QPrinter::DevicePixel);
    printer.setFullPage(true);
    paint(&printer);
  }
  else if(suffix == "svg")
  {
    QSvgGenerator printer;
    printer.setFileName(file);
    printer.setSize(size());
    paint(&printer);
  }
  else
  {
    if(suffix.isEmpty()) file += ".png";
    const QPixmap image = QPixmap::grabWidget(this);
    image.save(file);
  }
}

/// Save the upcoming states of this viewer as a video.
void
Viewer::save_video(bool value)
{
  if(value)
  {
    that->video_directory = QFileDialog::getExistingDirectory(this, that->action_save_video->text(), QString(), QFileDialog::ShowDirsOnly);
    if(that->video_directory.isNull())
    {
      that->action_save_video->setChecked(false);
    }
    else
    {
      that->video_counter = 0;
      that->video_timer = startTimer(100);
    }
  }
  else
  {
    killTimer(that->video_timer);
  }
}

/// Map a point from scene coordinates to viewport coordinates.
QPoint
Viewer::from_scene(qreal x, qreal y) const
{
  return that->transform.map(QPointF(x, y)).toPoint();
}

/// Map a rectangle from scene coordinates to viewport coordinates.
QRect
Viewer::from_scene(const QRectF &rect) const
{
  return that->transform.mapRect(rect).toRect();
}

/// Map a rectangle from viewport coordinates to scene coordinates.
QRectF
Viewer::to_scene(const QRect &rect) const
{
  return that->transform.inverted().mapRect(rect);
}

/// Map a point from viewport coordinates to scene coordinates.
QPointF
Viewer::to_scene(const QPoint &p) const
{
  return that->transform.inverted().map(p);
}

/// The bounding rectangle of the scene.
QRectF
Viewer::bounds(void) const
{
  QRectF rect;
  for(LayerList::const_reference layer: that->layers)
  {
    rect |= layer.second.bounds;
  }
  return rect;
}

void
Viewer::_show(bool value)
{
  that->layers[static_cast<const QAction *>(sender())->data().toInt()].visible = value;
}

/// \name QAbstractScrollArea Interface
/// \{

void Viewer::scrollContentsBy(int dx, int dy)
{
  that->controller().on_scroll(that, dx, dy);
}

void Viewer::closeEvent(QCloseEvent *)
{
  that->action_save_video->setChecked(false);
}

void Viewer::mouseMoveEvent(QMouseEvent *event)
{
  if(!that->mouse_dragging && (event->pos() - that->mouse_pos).manhattanLength() >= QApplication::startDragDistance())
  {
    that->mouse_dragging = true;
    setCursor(Qt::ClosedHandCursor);
  }
  if(that->mouse_dragging)
  {
    switch(that->mouse_buttons)
    {
      case Qt::RightButton:
      that->controller().on_drag_right(that, event);
      break;

      default:
      that->controller().on_drag(that, event);
    }
    that->mouse_pos = event->pos();
  }
}

void Viewer::mousePressEvent(QMouseEvent *event)
{
  that->mouse_dragging = false;
  that->mouse_pos = event->pos();
  that->mouse_buttons = event->buttons();
}

void Viewer::mouseReleaseEvent(QMouseEvent *event)
{
  setCursor(Qt::ArrowCursor);
  if(!that->mouse_dragging)
  {
    switch(that->mouse_buttons)
    {
      case Qt::RightButton:
      that->controller().on_click_right(that, event);
      break;

      default:
      that->controller().on_click(that, event);
    }
  }
}

// for delta = 120 (15° rotation) we want to zoom by 20%
// s = 1.2^(delta / 120) = (1.2^(1/120))^delta
void Viewer::wheelEvent(QWheelEvent* event)
{
  that->controller().on_wheel(that, event);
}

void Viewer::resizeEvent(QResizeEvent *)
{
  horizontalScrollBar()->setPageStep(viewport()->width());
  horizontalScrollBar()->setSingleStep(viewport()->width() / 20);
  verticalScrollBar()->setPageStep(viewport()->height());
  verticalScrollBar()->setSingleStep(viewport()->height() / 20);
  that->update();
}

void Viewer::keyPressEvent(QKeyEvent *event)
{
  switch(event->key())
  {
    case Qt::Key_Up:
    if(reticle_is_visible())
    {
      if(reticle_pos().y() > 0)
      {
        reticle_pos(reticle_pos() - QPoint(0, 1));
      }
    }
    break;

    case Qt::Key_Down:
    if(reticle_is_visible())
    {
      if(reticle_pos().y() < viewport()->height())
      {
        reticle_pos(reticle_pos() + QPoint(0, 1));
      }
    }
    break;

    case Qt::Key_Left:
    if(reticle_is_visible())
    {
      if(reticle_pos().x() > 0)
      {
        reticle_pos(reticle_pos() - QPoint(1, 0));
      }
    }
    break;

    case Qt::Key_Right:
    if(reticle_is_visible())
    {
      if(reticle_pos().x() < viewport()->width())
      {
        reticle_pos(reticle_pos() + QPoint(1, 0));
      }
    }
    break;
  }
}

void Viewer::paintEvent(QPaintEvent *)
{
  static_cast<QGLWidget *>(viewport())->makeCurrent();
  const int width = viewport()->width(), height = viewport()->height();

  glViewport(0, 0, width, height);
  glMatrixMode(GL_PROJECTION);
  that->controller().apply(that);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  double r, g, b, a;
  that->background_brush.color().getRgbF(&r, &g, &b, 0);
  glClearColor(r, g, b, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // ---------- DRAW 3D OBJECTS

  for(LayerList::const_reference layer: that->layers)
  {
    if(!layer.second.visible)
    {
      continue;
    }

    for(ContextList::const_reference context: layer.second.data)
    {
      (context.first.point_style == Round && context.first.pen.width() > 1 ? glEnable : glDisable)(GL_POINT_SMOOTH);
      context.first.pen.color().getRgbF(&r, &g, &b, &a);
      glPointSize(context.first.pen.width());
      glLineWidth(context.first.pen.width());
      glColor4d(r, g, b, a);

      for(const std::function<void()> &p: context.second.functions)
      {
        p();
      }
    }
  }

  // ---------- DRAW 2D OBJECTS

  paint(viewport());
}

void Viewer::paint(QPaintDevice *device)
{
  QPainter painter(device);
  painter.setRenderHints(0);
  painter.setTransform(that->transform);

  for(LayerList::const_reference layer: that->layers)
  {
    if(!layer.second.visible)
    {
      continue;
    }

    for(ContextList::const_reference context: layer.second.data)
    {
      painter.setPen(context.first.pen);
      painter.setBrush(context.first.brush);
      painter.setFont(context.first.font);

      // ---------- DRAW POINTS

      switch(context.first.point_style)
      {
        case Pixel:
        case Round:
        painter.drawPoints(context.second.points);
        break;

        case Cross:
        for(const QPointF &p: context.second.points)
        {
          static const QPointF x(5, 0), y(0, 5);
          painter.drawLine(p - x, p + x);
          painter.drawLine(p - y, p + y);
        }
        break;
      }

      // ---------- DRAW LINES

      painter.drawLines(context.second.lines);

      // ---------- DRAW RECTANGLES

      painter.drawRects(context.second.rectangles);

      // ---------- DRAW POLYGONS

      for(QPolygonF p: context.second.polygons)
      {
        painter.drawPolygon(p);
      }

      // ---------- DRAW ELLIPSES

      for(const Data::Ellipse &p: context.second.ellipses)
      {
        painter.drawEllipse(p.center, p.rx, p.ry);
      }

      // ---------- DRAW IMAGES

      for(const Data::Image &p: context.second.images)
      {
        painter.drawImage(p.position, p.image);
      }

      // ---------- DRAW TEXTS

      switch (context.first.font_direction)
      {
        case Horizontal:
        for(const Data::Text &p: context.second.texts)
        {
          painter.drawText(p.position, p.text);
        }
        break;

        case Vertical:
        for(const Data::Text &p: context.second.texts)
        {
          painter.save();
          painter.translate(p.position);
          painter.rotate(90);
          painter.drawText(QPointF(), p.text);
          painter.restore();
        }
        break;
      }

      // ---------- DRAW ARROWS

      for(const Data::Arrow &p: context.second.arrows)
      {
        static const qreal arrow_size = 10, angle = M_PI * 11 / 12;
        painter.drawLine(p.point, p.point + QPointF(std::cos(p.angle + angle), std::sin(p.angle + angle)) * arrow_size);
        painter.drawLine(p.point, p.point + QPointF(std::cos(p.angle - angle), std::sin(p.angle - angle)) * arrow_size);
      }
    }
  }

  // ---------- DRAW AXES

  painter.resetTransform();

  if(that->axes_enabled)
  {
    painter.setPen(that->axes_pen);
    painter.setFont(qApp->font());
    QFontMetrics metrics = painter.fontMetrics();

    switch(that->axes_type)
    {
      case AxesUV:
      that->draw_axes_uv(painter, metrics);
      break;

      case AxesXY:
      that->draw_axes_xy(painter, metrics);
      break;
    }
  }

  // ---------- DRAW RETICLE

  if(that->reticle_enabled)
  {
    painter.setPen(that->reticle_pen);
    painter.drawLine(0, that->reticle_pos.y(), viewport()->width(), that->reticle_pos.y());
    painter.drawLine(that->reticle_pos.x(), 0, that->reticle_pos.x(), viewport()->height());
  }
}

void Viewer::timerEvent(QTimerEvent *)
{
  const QPixmap image = QPixmap::grabWidget(this);
  const QString file = QString("%1/image_%2.png").arg(that->video_directory).arg(that->video_counter);
  image.save(file);
  ++that->video_counter;
}

/// \}
}}
