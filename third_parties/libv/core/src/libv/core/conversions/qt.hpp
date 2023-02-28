/**

\file
\author Alexis Wilhelm (2012)
\copyright 2012 Institut Pascal

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

#ifndef LIBV_CORE_CONVERSIONS_QT_HPP
#define LIBV_CORE_CONVERSIONS_QT_HPP

#include "wrapper.hpp"

#include <libv/core/found/qt5gui>
#if defined(LIBV_CORE_QT5GUI_FOUND) || !defined(LIBV_IGNORE_OPTIONAL_DEPENDENCIES)

#include <libv/core/image/image.hpp>
#include <QImage>

namespace v {
namespace core {
/// \addtogroup conversions
/// \{

/// Convert a QImage to an Image.
template<class T> Image<T>
convert(QImage &src, const Wrapper<Image<T> > *)
{
  return Image<T>(reinterpret_cast<typename Image<T>::pointer_to_scalar>(src.bits()), src.height(), src.width(), src.bytesPerLine() / sizeof(typename Image<T>::scalar_type));
}

/// Convert an Image to a QImage.
inline QImage
convert(const ImageU8r &src, const Wrapper<QImage> *)
{
  struct Private
  {
    static const QVector<QRgb> &
    init_colors(void)
    {
      static QVector<QRgb> colors;
      for(int i = 0; i < 256; ++i)
      {
        colors.push_back(qRgb(i, i, i));
      }
      return colors;
    }
  };
  static const QVector<QRgb> colors = Private::init_colors();
  QImage image((uchar *) src.data(), int(src.width()), int(src.height()), int(src.row_step()), QImage::Format_Indexed8);
  image.setColorTable(colors);
  return image;
}

/// Convert an Image to a QImage.
inline QImage
convert(const ImageRGBU8r &src, const Wrapper<QImage> *)
{
  return QImage((uchar *) src.data(), int(src.width()), int(src.height()), int(src.row_step()), QImage::Format_RGB888);
}

/// Convert an Image to a QImage.
inline QImage
convert(const ImageBGRAU8r &src, const Wrapper<QImage> *)
{
  return QImage((uchar *) src.data(), int(src.width()), int(src.height()), int(src.row_step()), QImage::Format_ARGB32);
}

/// \}
}}

#endif
#endif
