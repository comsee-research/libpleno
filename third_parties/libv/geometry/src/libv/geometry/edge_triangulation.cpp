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

#include "edge_triangulation.hpp"
#include "triangulation.hpp"

using namespace Eigen;

namespace v {
namespace geometry {

/**

Triangulate the point nearest to two rays and recover it's orientation using edge information.
The cross product is taken to obtain plane, the intersection of the two plane is the lines defined by a third cross product.
*
* //! This function has never been tested.
*
\return True if the operation succeeds.

*/
LIBV_GEOMETRY_EXPORT bool triangulate_edge_mid_point
( const Ray3d &ray1 ///< A ray. The direction does not need to be normalized.
, const Vector3d & edge_dir1 ///< The 3D direction of the edge in 3D for the point 1
, const Ray3d &ray2 ///< Idem
, const Vector3d & edge_dir2 ///< The 3D direction of the edge in 3D for the point 2
, Vector3d &point ///< The triangulated point.
, Vector3d &edge_dir ///< The triangulated orientation of the contour.
)
{
  if (triangulate_mid_point(ray1, ray2, point))
  {
    Vector3d n1 = ((point - ray1.origin).cross(edge_dir1));
    Vector3d n2 = ((point - ray2.origin).cross(edge_dir2));
    edge_dir = n1.cross(n2);
    edge_dir.normalize();
    return true;
  }
  return false;
}

}}
