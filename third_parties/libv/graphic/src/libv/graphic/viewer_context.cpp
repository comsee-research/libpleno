/**

\file
ViewerContext class definition.
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

#include <set>

#include "viewer_context.hpp"
#include "private/global.hpp"

namespace v {
namespace graphic {
namespace {

using namespace viewers_;

static int current_viewer_ = 0;
static std::set<int> used_viewer_handles_;
static std::set<std::pair<int, int> > clear_;

static uint32_t
to_int(v::RGBAU8 value)
{
  return qRgba(value.r(), value.g(), value.b(), value.a());
}

} // back in namespace v::graphic

/// \addtogroup viewers
/// \{

/// The most recently used viewer.
LIBV_GRAPHIC_EXPORT ViewerContext
viewer(void)
{
  start_viewers();
  std::lock_guard<std::mutex> lock(mutex_);
  return ViewerContext();
}

/// The viewer with the given identifier.
LIBV_GRAPHIC_EXPORT ViewerContext
viewer(int n)
{
  start_viewers();
  std::lock_guard<std::mutex> lock(mutex_);
  current_viewer_ = n;
  return ViewerContext();
}

/// A new viewer.
LIBV_GRAPHIC_EXPORT ViewerContext
new_viewer(void)
{
  start_viewers();
  std::lock_guard<std::mutex> lock(mutex_);
  for(current_viewer_ = 0; used_viewer_handles_.count(current_viewer_); ++current_viewer_);
  return ViewerContext();
}

/// \}

ViewerContext::ViewerContext(void)
  : layer_(0)
  , brush_color_(to_int(black))
  , brush_style_(NoBrush)
  , font_direction_(Horizontal)
  , font_family_(Times)
  , font_size_(10)
  , font_style_(Normal)
  , pen_color_(to_int(black))
  , pen_style_(SolidLine)
  , pen_width_(1)
  , point_style_(Pixel)
{
  viewer(current_viewer_);
}

/// The current viewer.
int
ViewerContext::viewer(void) const
{
  return viewer_;
}

/// The current layer.
int
ViewerContext::layer(void) const
{
  return layer_;
}

/**

\name Functions affecting the context.
The changes are local to this ViewerContext object.
You have to maintain a copy of this object if you want to make these changes persistent.

\{

*/

#define _DEFINE_ATTRIBUTE(_name, _args, _value)\
  /** \param value A new value. */\
  /** \returns \c *this so you can chain commands. */\
  ViewerContext &ViewerContext::_name _args\
  {\
    _name##_ = _value;\
    return *this;\
  }\

/// Switch to another viewer.
ViewerContext &ViewerContext::viewer(int viewer)
{
  viewer_ = current_viewer_ = viewer;
  used_viewer_handles_.insert(viewer);
  return *this;
}

/// Switch to another layer.
_DEFINE_ATTRIBUTE(layer, (int value), value)

/// Configure the pen's attributes.
_DEFINE_ATTRIBUTE(pen_width, (int value), value)

/// Configure the pen's attributes.
_DEFINE_ATTRIBUTE(pen_color, (const v::RGBAU8 &value), to_int(value))

/// Configure the pen's attributes.
/// \warning It is advised not to use this functionality as it may lead to really ugly plots.
_DEFINE_ATTRIBUTE(pen_style, (PenStyle value), value)

/// Configure the brush's attributes.
_DEFINE_ATTRIBUTE(brush_color, (const v::RGBAU8 & value), to_int(value))

/// Configure the brush's attributes.
/// \warning It is advised not to use this functionality as it may lead to really ugly plots.
_DEFINE_ATTRIBUTE(brush_style, (BrushStyle value), value)

/// Configure the font.
/// \warning It is advised not to use this functionality as it may lead to really ugly plots.
_DEFINE_ATTRIBUTE(font_family, (FontFamily value), value)

/// Configure the font.
/// \warning It is advised not to use this functionality as it may lead to really ugly plots.
_DEFINE_ATTRIBUTE(font_style, (FontStyle value), value)

/// Configure the font.
_DEFINE_ATTRIBUTE(font_direction, (FontDirection value), value)

/// Configure the font.
_DEFINE_ATTRIBUTE(font_size, (int value), value)

/// Configure the point style.
_DEFINE_ATTRIBUTE(point_style, (PointStyle value), value)

/// \}

ViewerContext &ViewerContext::update(void)
{
  std::lock_guard<std::mutex> lock(mutex_);
  commands_[viewer()].first.push_back(CommandList::value_type::create<Command<Commands::update> >(Commands::update()));
  std::vector<CommandList> &m = commands_[viewer()].second[layer()];

  const std::set<std::pair<int, int> >::iterator p = clear_.find(std::make_pair(viewer(), layer()));
  if(p != clear_.end())
  {
    m.clear();
    clear_.erase(p);
  }

  m.resize(m.size() + 1);
  m.back().swap(new_commands_[viewer()][layer()]);
  return *this;
}

ViewerContext &ViewerContext::clear(void)
{
  std::lock_guard<std::mutex> lock(mutex_);
  CommandList &m = new_commands_[viewer()][layer()];
  m.clear();
  const Commands::clear _new = {*this};
  m.push_back(CommandList::value_type::create<Command<Commands::clear> >(_new));
  clear_.insert(std::make_pair(viewer(), layer()));
  return *this;
}

ViewerContext &ViewerContext::add_circle(float x, float y, float r)
{
  return add_ellipse(x, y, r, r);
}

}}
