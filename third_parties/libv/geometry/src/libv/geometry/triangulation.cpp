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

#include "triangulation.hpp"

using namespace Eigen;

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/**

Triangulate the point nearest to two rays.

\return True if the operation succeeds.

*/
LIBV_GEOMETRY_EXPORT bool triangulate_mid_point
( const Ray3d &ray1 ///< A ray. The direction does not need to be normalized.
, const Ray3d &ray2 ///< Idem
, Eigen::Vector3d &point ///< The triangulated point.
)
{
  Matrix<double, 3, 2> m;
  m << ray1.direction, -ray2.direction;
  Vector2d r = (m.transpose() * m).inverse() * m.transpose() * (ray2.origin - ray1.origin);
  if(r.x() < 0 || r.y() < 0) return false;
  point = (r.x() * ray1.direction + ray1.origin + r.y() * ray2.direction + ray2.origin) / 2;
  return true;
}

/// \}
}}
