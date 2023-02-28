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

#include <QMenu>
#include <memory>
#include <cmath>
#include <boost/preprocessor.hpp>

#include "global.hpp"
#include "viewer_data.hpp"

namespace v {
namespace graphic {
namespace {

using namespace viewers_;

#define _DEFINE_COMMAND(_name, _args, _value)\
  _DEFINE_COMMAND_OVERLOAD(_name, _args, _value)\
  _DEFINE_COMMAND_IMPLEMENTATION(_name)

#define _DEFINE_COMMAND_IMPLEMENTATION(_name)\
  void Commands::_name::operator()(Viewer *viewer)

#define _DEFINE_COMMAND_OVERLOAD(_name, _args, _value)\
\
  ViewerContext &\
  ViewerContext::_name _args\
  {\
    std::lock_guard<std::mutex> lock(mutex_);\
    _COMMANDS.push_back(CommandList::value_type::create<Command<Commands::_name> >(construct<Commands::_name> _value));\
    return *this;\
  }\

#define _DEFINE_CONSTRUCTOR(z, n, data)\
\
  template<class T, BOOST_PP_ENUM_PARAMS(n, class T)>\
  static T construct(BOOST_PP_ENUM_BINARY_PARAMS(n, const T, &a))\
  {\
    T a = {BOOST_PP_ENUM_PARAMS(n, a)};\
    return a;\
  }\

BOOST_PP_REPEAT_FROM_TO(1, 10, _DEFINE_CONSTRUCTOR, :)

#define _THIS (*viewer)
#define _THAT (*_THIS.that)
#define _LAYER (_THAT.layers[context.layer()])
#define _CONTEXT (_LAYER.data[context])
#define _COMMANDS (commands_[viewer()].first)

#define _(y) (viewer->that->axes_type == AxesXY ? -(y) : (y))

static uint32_t to_int(v::RGBAU8 value)
{
  return qRgba(value.r(), value.g(), value.b(), value.a());
}

static v::ImageBGRAU8 to_bgra(const v::ImageRGBAU8cr &rgba)
{
  v::ImageBGRAU8 bgra(rgba.height(), rgba.width());
  V_FOR_EACH_PIXEL(p, rgba)
  {
    bgra[p.row][p.column].r(p->r()).g(p->g()).b(p->b()).a(p->a());
  }
  return bgra;
}

}

_DEFINE_COMMAND(size, (int width, int height), (width, height))
{
  _THIS.window()->resize(_THIS.window()->size() - _THIS.size() + QSize(width, height));
  _THAT.has_size = true;
}

_DEFINE_COMMAND(look_at, (float x, float y), (x, y))
{
  _THIS.look_at(QPointF(x, _(y)));
}

_DEFINE_COMMAND(zoom, (float zoom), (zoom))
{
  _THIS.zoom(zoom);
}

_DEFINE_COMMAND(title, (const std::string &title), (title))
{
  QString title = QString::fromUtf8(title_.c_str());
  _THIS.setWindowTitle(title);
  _THAT.window->setWindowTitle(title);
  _THAT.dock->setWindowTitle(title);
  _THAT.title->setText(title);
}

_DEFINE_COMMAND(axes, (ViewerAxesType value), (value))
{
  _THAT.axes_type = value;
}

_DEFINE_COMMAND(background, (const v::RGBAU8 &value), (to_int(value)))
{
  _THAT.background_brush.setColor(QColor::fromRgba(value));
}

_DEFINE_COMMAND(on_click, (const std::function<void(float, float)> &callback), (callback))
{
  _THAT.on_click_ = callback;
}

_DEFINE_COMMAND_IMPLEMENTATION(update)
{
  _THAT.update();
}

_DEFINE_COMMAND_IMPLEMENTATION(clear)
{
  _LAYER.clear();
}

_DEFINE_COMMAND(show, (bool value), (*this, value))
{
  if(_LAYER.action_visibility)
  {
    _LAYER.action_visibility->setChecked(value);
  }
  else
  {
    _LAYER.visible = value;
  }
}

_DEFINE_COMMAND(name, (const std::string &value), (*this, value))
{
  if(_LAYER.action_visibility)
  {
    _LAYER.action_visibility->setText(value.c_str());
  }
  else
  {
    _LAYER.action_visibility.reset(_THAT.menu_visibility->menu()->addAction(value.c_str()));
    _LAYER.action_visibility->setCheckable(true);
    _LAYER.action_visibility->setChecked(_LAYER.visible);
    _LAYER.action_visibility->setData(context.layer());
    _THIS.connect(_LAYER.action_visibility.get(), SIGNAL(toggled(bool)), SLOT(_show(bool)));
    _THAT.menu_visibility->setVisible(true);
  }
}

_DEFINE_COMMAND(interaction_mode, (InteractionMode value), (value))
{
  _THAT.controllers.actions()[value]->setChecked(true);
}

#undef _COMMANDS
#define _COMMANDS (new_commands_[viewer()][layer()])

_DEFINE_COMMAND(add_point, (float x, float y), (*this, x, y))
{
  const QPointF p(x, _(y));
  _CONTEXT.points.append(p);
  _LAYER.grow(p);
}

_DEFINE_COMMAND(add_line, (float x1, float y1, float x2, float y2), (*this, x1, y1, x2, y2))
{
  const QLineF a(x1, _(y1), x2, _(y2));
  _CONTEXT.lines.append(a);
  _LAYER.grow(a.p1());
  _LAYER.grow(a.p2());
}

_DEFINE_COMMAND(add_rect, (float x, float y, float width, float height), (*this, x, y, width, height))
{
  const QRectF a(x, _(y), width, height);
  _CONTEXT.rectangles.append(a);
  _LAYER.grow(a);
}

_DEFINE_COMMAND(add_ellipse, (float x, float y, float rx, float ry), (*this, x, y, rx, ry))
{
  const Data::Ellipse data = {QPointF(x, _(y)), rx, ry};
  _CONTEXT.ellipses.append(data);
  const QPointF r(rx, ry);
  _LAYER.grow(data.center + r);
  _LAYER.grow(data.center - r);
}

_DEFINE_COMMAND(add_triangle, (float x1, float y1, float x2, float y2, float x3, float y3), (*this, x1, y1, x2, y2, x3, y3))
{
  QPolygonF p(3);

  p[0].setX(x1);
  p[0].setY(_(y1));

  p[1].setX(x2);
  p[1].setY(_(y2));

  p[2].setX(x3);
  p[2].setY(_(y3));

  _CONTEXT.polygons.append(p);
  _LAYER.grow(p.boundingRect());
}

_DEFINE_COMMAND(add_quad, (float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4), (*this, x1, y1, x2, y2, x3, y3, x4, y4))
{
  QPolygonF p(4);

  p[0].setX(x1);
  p[0].setY(_(y1));

  p[1].setX(x2);
  p[1].setY(_(y2));

  p[2].setX(x3);
  p[2].setY(_(y3));

  p[3].setX(x4);
  p[3].setY(_(y4));

  _CONTEXT.polygons.append(p);
  _LAYER.grow(p.boundingRect());
}

_DEFINE_COMMAND(add_arrow, (float x1, float y1, float x2, float y2, bool both), (*this, x1, y1, x2, y2, both))
{
  const QLineF a(x1, _(y1), x2, _(y2));
  _CONTEXT.lines.append(a);
  _LAYER.grow(a.p1());
  _LAYER.grow(a.p2());

  const Data::Arrow head = {QPointF(x2, _(y2)), std::atan2(_(y2 - y1), x2 - x1)};
  _CONTEXT.arrows.append(head);
  if(both)
  {
    const Data::Arrow tail = {QPointF(x1, _(y1)), head.angle + M_PI};
    _CONTEXT.arrows.append(tail);
  }
}

_DEFINE_COMMAND(add_image, (float x, float y, const v::ImageU8cr &image), (*this, x, y, (std::make_shared<v::Convertible_<v::ImageU8, QImage> >(image))))
{
  const Data::Image data = {QPointF(x, _(y)), image->convert().convertToFormat(QImage::Format_RGB32)};
  _CONTEXT.images.append(data);
  _LAYER.grow(QRectF(data.image.rect()).translated(data.position));
}

_DEFINE_COMMAND_OVERLOAD(add_image, (float x, float y, const v::ImageRGBU8cr &image), (*this, x, y, (std::make_shared<v::Convertible_<v::ImageRGBU8, QImage> >(image))))

_DEFINE_COMMAND_OVERLOAD(add_image, (float x, float y, const v::ImageRGBAU8cr &image), (*this, x, y, (std::make_shared<v::Convertible_<v::ImageRGBAU8, QImage> >(to_bgra(image)))))

_DEFINE_COMMAND(add_text, (float x, float y, const std::string &text), (*this, x, y, text))
{
  const Data::Text data = {QPointF(x, _(y)), QString::fromUtf8(text.c_str())};
  _CONTEXT.texts.append(data);

  QFont font;
  font.setPixelSize(context.font_size_);
  switch(context.font_style_)
  {
    case Normal: break;
    case Bold: font.setBold(true); break;
    case Italic: font.setItalic(true); break;
  }
  switch(context.font_family_)
  {
    case Arial: font.setFamily("Arial"); break;
    case Times: font.setFamily("Times"); break;
    case Courier: font.setFamily("Courier"); break;
  }
  QFontMetrics metrics(font);
  const int height = metrics.height();
  const int width = metrics.width(data.text);
  switch(context.font_direction_)
  {
    case Horizontal: _LAYER.grow(QRectF(data.position.x(), data.position.y() - height, width, height)); break;
    case Vertical: _LAYER.grow(QRectF(data.position.x(), data.position.y(), height, width)); break;
  }
}

_DEFINE_COMMAND(add_opengl, (const std::function<void()> &function), (*this, function))
{
  _CONTEXT.functions.append(function);
}

}}
