/**

\file
Global declarations.
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

#include <libv/graphic/export.hpp>
#include <string>

class QDockWidget;
class QMainWindow;
class QMenu;

namespace v
{
  /// Libv graphic classes.
  /// This namespace contains all the classes provided by the \e graphic module.
  namespace graphic
  {
    namespace viewers_
    {
      struct ViewerData;
      union Commands;
    }
    struct Viewer;
    struct ViewerContext;
    LIBV_GRAPHIC_EXPORT void start_viewers(void);
    LIBV_GRAPHIC_EXPORT void start_viewers(int &, char **, const std::string &, const std::string &);
    LIBV_GRAPHIC_EXPORT void stop_viewers(void);
    LIBV_GRAPHIC_EXPORT void wait_viewers(void);
    LIBV_GRAPHIC_EXPORT void set_interval(int);
    LIBV_GRAPHIC_EXPORT void use_single_window(bool);
    LIBV_GRAPHIC_EXPORT ViewerContext viewer(void);
    LIBV_GRAPHIC_EXPORT ViewerContext viewer(int);
    LIBV_GRAPHIC_EXPORT ViewerContext new_viewer(void);
  }
  using namespace graphic;
}
