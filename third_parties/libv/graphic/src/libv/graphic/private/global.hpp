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

#ifndef LIBV_GRAPHIC_PRIVATE_GLOBAL_HPP
#define LIBV_GRAPHIC_PRIVATE_GLOBAL_HPP

#include <mutex>
#include <libv/core/memory/polymorphic.hpp>

#include "commands.hpp"

namespace v {
namespace graphic {
namespace viewers_ {

/// A list of commands.
typedef std::vector<v::Polymorphic<AbstractCommand, sizeof(Commands)> > CommandList;

/// A list of layer-scope commands.
typedef std::map<int, std::vector<CommandList> > LayerCommandList;

/// A list of viewer-scope commands.
typedef std::map<int, std::pair<CommandList, LayerCommandList> > ViewerCommandList;

/// Commands validated but not executed.
extern ViewerCommandList commands_;

extern std::map<int, std::map<int, CommandList> > new_commands_;

/// This mutex protects the variables shared by \ref system.cpp and \ref viewer_context.cpp.
extern std::mutex mutex_;

}}}

#endif
