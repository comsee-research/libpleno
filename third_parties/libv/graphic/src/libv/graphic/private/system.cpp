/**

\file
Viewer-related machinery.
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

#include <QApplication>
#include <QMenu>
#include <QSettings>
#include <libv/graphic/viewer_context.hpp>

#include "global.hpp"
#include "system.hpp"

/*
  « If you have resources in a library, you need to force initialization of your resources by calling Q_INIT_RESOURCE() with the base name of the .qrc file. [...] This ensures that the resources are linked into the final application binary in the case of static linking. »
  https://doc.qt.io/qt-5/resources.html#using-resources-in-a-library
*/
static void init_qt()
{
  Q_INIT_RESOURCE(icons);
}

namespace v {
namespace graphic {
namespace viewers_ {
namespace {

typedef std::map<int, Viewer *> ViewerList;

static void save(QSettings &s, QMainWindow *w, QObject *o)
{
  s.setValue("state", w->saveState());
  s.setValue("size", w->size());
  s.setValue("pos", w->pos());

  if(!o) return;
  for(QAction *a: o->findChildren<QAction *>())
    s.setValue(a->objectName(), a->isChecked());
}

static void load(QSettings &s, QMainWindow *w, QObject *o)
{
  QVariant v;
  if(!(v = s.value("state")).isNull()) w->restoreState(v.toByteArray());
  if(!(v = s.value("size")).isNull()) w->resize(v.toSize());
  if(!(v = s.value("pos")).isNull()) w->move(v.toPoint());

  if(!o) return;
  for(QAction *a: o->findChildren<QAction *>())
  {
    if(!(v = s.value(a->objectName())).isNull())
      a->setChecked(v.toBool());
  }
}

static void save(QSettings &s, ViewerList::reference p, QObject *o)
{
  p.second->as_window();
  s.beginWriteArray("viewers");
  s.setArrayIndex(p.first);
  save(s, static_cast<QMainWindow *>(p.second->window()), o);
}

static void load(QSettings &s, ViewerList::reference p, QObject *o)
{
  p.second->as_window();
  s.beginReadArray("viewers");
  s.setArrayIndex(p.first);
  load(s, static_cast<QMainWindow *>(p.second->window()), o);
}

template<class T>
static void save(T &x, QObject *o = 0)
{
  QSettings s;
  s.beginGroup("libv");
  save(s, x, o);
}

template<class T>
static void load(T &x, QObject *o = 0)
{
  QSettings s;
  s.beginGroup("libv");
  load(s, x, o);
}

}

System::System(std::condition_variable &condition)
  : stop(true)
  , interval(-1)
  , condition(condition)
{
  moveToThread(qApp->thread());
  setParent(qApp);
  qApp->postEvent(this, new QEvent(QEvent::User));
}

// continue initialization from the Qt thread
void
System::customEvent(QEvent *)
{
  init_qt();
  window = new QMainWindow;
  window->setWindowTitle(qApp->applicationName());
  ui.setupUi(window);
  connect(qApp, SIGNAL(aboutToQuit()), SLOT(finalize()));
  connect(ui.action_single_window_mode, SIGNAL(toggled(bool)), SLOT(single_window_mode(bool)));
  load(window, window);
  stop = false;
  timer = startTimer(100);
  condition.notify_one();
}

void
System::timerEvent(QTimerEvent *)
{
  if(stop)
  {
    qApp->quit();
  }

  if(interval >= 0)
  {
    killTimer(timer);
    timer = startTimer(interval);
    interval = -1;
  }

  ViewerCommandList commands;
  std::unique_lock<std::mutex> lock(mutex_);
  commands.swap(commands_);
  lock.unlock();

  for(ViewerCommandList::const_reference viewer: commands)
  {
    std::map<int, Viewer *>::iterator p = viewers.find(viewer.first);

    if(p == viewers.end())
    {
      p = viewers.insert(p, std::make_pair(viewer.first, new Viewer));
      p->second->menu()->addSeparator();
      p->second->menu()->addAction(ui.action_single_window_mode);
      window->addDockWidget(Qt::TopDockWidgetArea, p->second->as_dock());
      load(*p, p->second);
      if(ui.action_single_window_mode->isChecked())
      {
        p->second->as_dock();
        load(window);
      }
    }

    Viewer *w = p->second;
    w->window()->show();

    for(CommandList::const_reference p: viewer.second.first)
    {
      (*p)(w);
    }
    for(LayerCommandList::const_reference layer: viewer.second.second)
    {
      for(const CommandList &commands: layer.second)
      {
        for(CommandList::const_reference p: commands)
        {
          (*p)(w);
        }
      }
    }
  }
}

void System::finalize()
{
  save(window, window);
  window->deleteLater();

  for(ViewerList::reference p: viewers)
  {
    save(p, p.second);
    p.second->deleteLater();
  }
}

void System::single_window_mode(bool value)
{
  if(value)
  {
    for(ViewerList::reference p: viewers)
    {
      save(p);
      p.second->as_dock();
    }

    load(window);
  }
  else
  {
    save(window);

    for(ViewerList::reference p: viewers)
    {
      load(p);
    }
  }

  window->setVisible(value);
}

}}}
