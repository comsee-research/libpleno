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

#ifndef LIBV_GRAPHIC_PRIVATE_VIEWER_DATA_HPP
#define LIBV_GRAPHIC_PRIVATE_VIEWER_DATA_HPP

#include <QActionGroup>

#include "controller_2d.hpp"
#include "controller_cad.hpp"
#include "layer.hpp"
#include "ui_viewer.h"

namespace v {
namespace graphic {
namespace viewers_ {

typedef std::map<int, Layer> LayerList;

/// Private members.
struct ViewerData
: Ui_Viewer
{
  ViewerData(Viewer *);
  Viewer *that;
  LayerList layers;
  bool axes_enabled, grid_enabled, reticle_enabled;
  QPen axes_pen, grid_pen, reticle_pen;
  QBrush background_brush;
  QPoint reticle_pos;
  ViewerAxesType axes_type;
  QTransform transform;
  std::function<void(float, float)> on_click_;
  int video_timer;
  QString video_directory;
  int video_counter;
  bool mouse_dragging;
  QPoint mouse_pos;
  Qt::MouseButtons mouse_buttons;
  void draw_axes_uv(QPainter &, const QFontMetrics &);
  void draw_axes_xy(QPainter &, const QFontMetrics &);
  void update(void);
  bool update_guard;
  bool has_size;
  Controller2D controller_2d;
  ControllerCAD controller_cad;
  QActionGroup controllers;
  Controller &controller();
};

}}}

#endif
