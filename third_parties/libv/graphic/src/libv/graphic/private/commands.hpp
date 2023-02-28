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

#ifndef LIBV_GRAPHIC_PRIVATE_COMMANDS_HPP
#define LIBV_GRAPHIC_PRIVATE_COMMANDS_HPP

#include <libv/core/conversions/qt.hpp>
#include <functional>
#include <memory>

#include "../viewer_context.hpp"

/// Declare a command.
#define _DECLARE_COMMAND(_name, _args)\
\
  struct _name\
  {\
    void operator()(Viewer *);\
    _args;\
  };\
\
  char _name##_[sizeof(Command<_name>)];\

namespace v {
namespace graphic {
namespace viewers_ {

/// A common base class for all commands.
struct AbstractCommand
{
  /// Execute this command.
  virtual void operator()(Viewer *) = 0;
  virtual ~AbstractCommand(void) {}
};

/// A command.
template<class T>
struct Command: AbstractCommand, T
{
  /// Constructor.
  Command(const T &other): T(other) {}
  void operator()(Viewer *v) { return T::operator()(v); }
};

/// Contains data for all known commands.
/// This union has the size of the largest subclass of AbstractCommand.
union Commands
{
  /// \privatesection
  _DECLARE_COMMAND(update, enum {nothing})
  _DECLARE_COMMAND(size, int width; int height)
  _DECLARE_COMMAND(look_at, float x; float y)
  _DECLARE_COMMAND(zoom, float zoom)
  _DECLARE_COMMAND(title, std::string title_)
  _DECLARE_COMMAND(axes, ViewerAxesType value)
  _DECLARE_COMMAND(background, uint32_t value)
  _DECLARE_COMMAND(on_click, std::function<void(float, float)> callback)
  _DECLARE_COMMAND(interaction_mode, int value)
  _DECLARE_COMMAND(clear, ViewerContext context)
  _DECLARE_COMMAND(show, ViewerContext context; bool value)
  _DECLARE_COMMAND(name, ViewerContext context; std::string value)
  _DECLARE_COMMAND(add_point, ViewerContext context; float x; float y)
  _DECLARE_COMMAND(add_arrow, ViewerContext context; float x1; float y1; float x2; float y2; bool both)
  _DECLARE_COMMAND(add_line, ViewerContext context; float x1; float y1; float x2; float y2)
  _DECLARE_COMMAND(add_ellipse, ViewerContext context; float x; float y; float rx; float ry)
  _DECLARE_COMMAND(add_rect, ViewerContext context; float x; float y; float width; float height)
  _DECLARE_COMMAND(add_triangle, ViewerContext context; float x1; float y1; float x2; float y2; float x3; float y3)
  _DECLARE_COMMAND(add_quad, ViewerContext context; float x1; float y1; float x2; float y2; float x3; float y3; float x4; float y4)
  _DECLARE_COMMAND(add_image, ViewerContext context; float x; float y; std::shared_ptr<v::Convertible<QImage> > image)
  _DECLARE_COMMAND(add_text, ViewerContext context; float x; float y; std::string text)
  _DECLARE_COMMAND(add_opengl, ViewerContext context; std::function<void()> function)
};

}}}

#undef _DECLARE_COMMAND
#endif
