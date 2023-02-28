/**

\file
\author Clement Deymier (2013)
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

#ifndef LIBV_GEOMETRY_TRIANGULATION_HPP
#define LIBV_GEOMETRY_TRIANGULATION_HPP

#include "ray.hpp"

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

LIBV_GEOMETRY_EXPORT bool triangulate_mid_point(const Ray3d &, const Ray3d &, Eigen::Vector3d &);

/// \}
}}

#endif
