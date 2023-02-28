/**

\file
\author Alexis Wilhelm (2012-2013)
\copyright 2012-2013 Institut Pascal

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

#ifndef LIBV_CORE_SERIALIZATION_EIGEN_HPP
#define LIBV_CORE_SERIALIZATION_EIGEN_HPP

#include "archives/base.hpp"

#include <libv/core/found/eigen>
#if defined(LIBV_CORE_EIGEN_FOUND) || !defined(LIBV_IGNORE_OPTIONAL_DEPENDENCIES)

#include <Eigen/Core>

namespace v {
namespace core {
/// \addtogroup serialization_eigen
/// \{

/**

Load matrices from an input archive.

*/
template<class T>
void load_eigen
( InputArchive &archive ///< An archive.
, T *values ///< A list of matrices.
, const size_t *extents ///< A list of extents.
, size_t rank ///< The rank of the array.
)
{
  load(archive, values->data(), extents, rank + 2);
}

/**

Save matrices in an output archive.

*/
template<class T>
void save_eigen
( OutputArchive &archive ///< An archive.
, const T *values ///< A list of matrices.
, size_t *extents ///< A list of extents.
, size_t rank ///< The rank of the array.
)
{
  extents[rank+0] = values->outerSize();
  extents[rank+1] = values->innerSize();
  save(archive, values->data(), extents, rank + 2);
}

#define _OVERLOAD(_type)\
\
  template<class T, int n1, int n2, int n3, int n4, int n5>\
  struct SerializableTraits<_type<T, n1, n2, n3, n4, n5> >: new_serializable_traits<T, 2, n1 * n2 * (n1 != Eigen::Dynamic) * (n2 != Eigen::Dynamic)> {};\
\
  /** \copydoc load_eigen() */\
  template<class T, int n1, int n2, int n3, int n4, int n5>\
  void load(/** An archive. */ InputArchive &archive, /** A list of matrices. */ _type<T, n1, n2, n3, n4, n5> *values, /** A list of extents. */ const size_t *extents, /** The rank of the array. */ size_t rank, typename std::enable_if<n1 != Eigen::Dynamic && n2 != Eigen::Dynamic>::type * = nullptr)\
  {\
    load_eigen(archive, values, extents, rank);\
  }\
\
  /** \copydoc save_eigen() */\
  template<class T, int n1, int n2, int n3, int n4, int n5>\
  void save(/** An archive. */ OutputArchive &archive, /** A list of matrices. */ const _type<T, n1, n2, n3, n4, n5> *values, /** A list of extents. */ size_t *extents, /** The rank of the array. */ size_t rank, typename std::enable_if<n1 != Eigen::Dynamic && n2 != Eigen::Dynamic>::type * = nullptr)\
  {\
    save_eigen(archive, values, extents, rank);\
  }\

_OVERLOAD(Eigen::Array)
_OVERLOAD(Eigen::Matrix)

/// \}
}}

#undef _OVERLOAD
#endif
#endif
