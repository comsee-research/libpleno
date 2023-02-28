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

#ifndef LIBV_CORE_CONVERSIONS_OPENCV_HPP
#define LIBV_CORE_CONVERSIONS_OPENCV_HPP

#include "wrapper.hpp"

#include <libv/core/found/opencv>
#if defined(LIBV_CORE_OPENCV_FOUND) || !defined(LIBV_IGNORE_OPTIONAL_DEPENDENCIES)

#include <libv/core/image/image.hpp>
#include <opencv/cv.h>

namespace v {
namespace core {
/// \addtogroup conversions
/// \{

/// Declare a conversion from an image container to another.
/// \param format_libv A color format with our notation. Can be U8, V3F32, etc.
/// \param format_opencv A color format with OpenCV's notation. Can be 8UC1, 32FC3, etc.
#define _DECLARE_CONVERSION(format_libv, format_opencv)\
\
  /** Interpret a Libv format_libv image as an OpenCV format_opencv image. */\
  /** \param src A Libv format_libv image. */\
  /** \return An OpenCV format_opencv image. */\
  inline cv::Mat\
  convert(const Image##format_libv##r &src, const Wrapper<cv::Mat> *)\
  {\
    return cv::Mat(int(src.height()), int(src.width()), CV_##format_opencv, src.data(), src.row_step() * sizeof(format_libv::scalar_type));\
  }\

/// Convert a cv::Mat to an Image.
template<class T> Image<T>
convert(cv::Mat &src, const Wrapper<Image<T> > *)
{
  return Image<T>(reinterpret_cast<typename Image<T>::pointer_to_scalar>(src.data), src.rows, src.cols, src.step / sizeof(typename Image<T>::scalar_type));
}

_DECLARE_CONVERSION(U8, 8UC1)
_DECLARE_CONVERSION(V3U8, 8UC3)
_DECLARE_CONVERSION(V4U8, 8UC4)

/// \}
}}

#undef _DECLARE_CONVERSION
#endif
#endif
