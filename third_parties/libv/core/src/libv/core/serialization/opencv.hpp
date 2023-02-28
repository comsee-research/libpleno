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

#ifndef LIBV_CORE_SERIALIZATION_OPENCV_HPP
#define LIBV_CORE_SERIALIZATION_OPENCV_HPP

#include "archives/base.hpp"

#include <libv/core/found/opencv>
#if defined(LIBV_CORE_OPENCV_FOUND) || !defined(LIBV_IGNORE_OPTIONAL_DEPENDENCIES)

#include <opencv2/core/core.hpp>

namespace v {
namespace core {
/// \addtogroup serialization_opencv
/// \{

template<class T>
struct SerializableTraits<cv::Size_<T> >: new_serializable_traits<T, 1, 2> {};

/**

Load sizes from an input archive.

*/
template<class T>
void load
( InputArchive &archive ///< An archive.
, cv::Size_<T> *values ///< A list of sizes.
, const size_t *extents ///< A list of extents.
, size_t rank ///< The rank of the array.
)
{
  load(archive, &values->width, extents, rank + 1);
}

/**

Save sizes in an output archive.

*/
template<class T>
void save
( OutputArchive &archive ///< An archive.
, const cv::Size_<T> *values ///< A list of sizes.
, size_t *extents ///< A list of extents.
, size_t rank ///< The rank of the array.
)
{
  extents[0] = 2;
  save(archive, &values->width, extents, rank + 1);
}

/// \}
}}

#endif
#endif
