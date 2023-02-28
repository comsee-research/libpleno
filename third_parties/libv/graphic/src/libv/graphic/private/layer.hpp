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

#ifndef LIBV_GRAPHIC_PRIVATE_LAYER_HPP
#define LIBV_GRAPHIC_PRIVATE_LAYER_HPP

#include <memory>
#include <map>

#include "data.hpp"
#include "style.hpp"

namespace v {
namespace graphic {
namespace viewers_ {

typedef std::map<Viewer::Style, Data> ContextList;

struct Layer
{
  ContextList data;
  QRectF bounds;
  bool visible;
  std::shared_ptr<QAction> action_visibility;
  void clear(void);
  void grow(const QPointF &);
  void grow(const QRectF &);
  Layer(void);
};

}}}

#endif
