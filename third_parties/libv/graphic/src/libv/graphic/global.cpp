/**

\file
Viewer-related machinery.
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
#include <QFileInfo>
#include <thread>

#include "private/global.hpp"
#include "private/system.hpp"

namespace v {
namespace graphic {
namespace {

using namespace viewers_;

static std::thread thread_;
static System *system_ = 0;

static void
start(int *argc, char **argv, std::condition_variable *condition)
{
  std::unique_lock<std::mutex> lock(mutex_);
  QApplication app(*argc, argv);
  condition->notify_one();
  lock.unlock();
  app.exec();
}

}

/// \addtogroup viewers
/// \{

/**

Initialize the viewer system.
You normally don't have to call this function yourself.

*/
LIBV_GRAPHIC_EXPORT void start_viewers()
{
  static int argc = 1;
  static char *argv[] = {(char *) ""};
  static std::string empty;
  start_viewers(argc, argv, empty, empty);
}

/// \copydoc start_viewers
LIBV_GRAPHIC_EXPORT void start_viewers
( int &argc ///< The argument count.
, char **argv ///< The argument list.
, const std::string &application_name ///< The name of your application.
, const std::string &organization_name ///< Your name.
)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if(!system_)
  {
    std::condition_variable condition;
    if(!qApp)
    {
      thread_ = std::thread(std::bind(start, &argc, argv, &condition));
      while(!qApp)
      {
        condition.wait(lock);
      }
      if(application_name.empty())
      {
        char name[200];
      #if defined(Q_OS_UNIX)
        int size = readlink("/proc/self/exe", name, sizeof(name));
      #elif defined(Q_OS_WIN)
        int size = GetModuleFileName(0, name, sizeof(name));
      #endif
        qApp->setApplicationName(QFileInfo(QString::fromLocal8Bit(name, size)).baseName());
      }
      else
      {
        qApp->setApplicationName(application_name.c_str());
      }
      if(organization_name.empty())
      {
        qApp->setOrganizationName("libv");
      }
      else
      {
        qApp->setOrganizationName(organization_name.c_str());
      }
    }
    system_ = new System(condition);
    while(system_->stop)
    {
      condition.wait(lock);
    }
  }
}

/**

Terminate the viewer system.

*/
LIBV_GRAPHIC_EXPORT void stop_viewers()
{
  system_->stop = true;
  wait_viewers();
}

/**

Block until all viewers are closed.

*/
LIBV_GRAPHIC_EXPORT void wait_viewers()
{
  if (thread_.joinable())
    thread_.join();
}

/**

Set the time interval between two consecutive updates of the viewers.
The default value is 100 ms.

*/
LIBV_GRAPHIC_EXPORT void set_interval
( int interval ///< In milliseconds.
)
{
  system_->interval = interval;
}

/**

Toggle the use of the single window mode.
In single window mode, all viewers are docked in a main window.
The default is false.

*/
LIBV_GRAPHIC_EXPORT void use_single_window
( bool value ///< True to use the single window mode.
)
{
  system_->ui.action_single_window_mode->setChecked(value);
}

/// \}
}}
