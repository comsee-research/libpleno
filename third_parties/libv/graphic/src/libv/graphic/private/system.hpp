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

#ifndef LIBV_GRAPHIC_PRIVATE_SYSTEM_HPP
#define LIBV_GRAPHIC_PRIVATE_SYSTEM_HPP

#include <condition_variable>
#include <libv/graphic/viewer.hpp>

#include "ui_system.h"

namespace v {
namespace graphic {
namespace viewers_ {

struct System
: QObject
{
  Q_OBJECT
  void timerEvent(QTimerEvent *);
  void customEvent(QEvent *);
public:
  System(std::condition_variable &);
  Q_SLOT void single_window_mode(bool);
  Q_SLOT void finalize();
  bool stop;
  int timer, interval;
  std::map<int, Viewer *> viewers;
  QMainWindow *window;
  Ui::Window ui;
  std::condition_variable &condition;
};

}}}

#endif
