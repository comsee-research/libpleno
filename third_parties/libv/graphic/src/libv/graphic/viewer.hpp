/**

\file
Viewer class definition.
This header should be included by the adventurer.
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

#ifndef LIBV_GRAPHIC_VIEWER_HPP
#define LIBV_GRAPHIC_VIEWER_HPP

#include <QAbstractScrollArea>
#include <libv/core/conversions/wrapper.hpp>

#include "types.hpp"

namespace v {
namespace graphic {
/// \addtogroup viewers
/// \{

/// A viewer.
struct Viewer
  : QAbstractScrollArea
{
  Q_OBJECT
public:
  struct Style;
  Viewer(QWidget * = 0);
  ~Viewer(void);
  bool reticle_is_visible(void) const;
  QPoint reticle_pos(void) const;
  QRectF bounds(void) const;
  QPoint from_scene(qreal, qreal) const;
  QRect from_scene(const QRectF &) const;
  QRectF to_scene(const QRect &) const;
  QPointF to_scene(const QPoint &) const;
  QMainWindow *as_window();
  QDockWidget *as_dock();
  QMenu *menu();
public Q_SLOTS:
  void axes_are_visible(bool);
  void grid_is_visible(bool);
  void reticle_is_visible(bool);
  void reticle_pos(const QPoint &);
  void save_screenshot(void);
  void save_video(bool);
  void zoom_fit_best(void);
  void zoom_original(void);
  void zoom(float);
  void look_at(QPointF);
private Q_SLOTS:
  void _show(bool);
private:
  viewers_::ViewerData *that;
  friend union viewers_::Commands;
  void paint(QPaintDevice *);

  void closeEvent(QCloseEvent *);
  void keyPressEvent(QKeyEvent *);
  void mouseMoveEvent(QMouseEvent *);
  void mousePressEvent(QMouseEvent *);
  void mouseReleaseEvent(QMouseEvent *);
  void paintEvent(QPaintEvent *);
  void resizeEvent(QResizeEvent *);
  void scrollContentsBy(int, int);
  void timerEvent(QTimerEvent *);
  void wheelEvent(QWheelEvent *);
};

/// \}
}}

#endif
