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

#include "camera_model.hpp"
#include "epipolar_distance.hpp"

using namespace Eigen;

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/**

Compute epipolar line coefficients.

\return
The \e a, \e b, \e c coefficients of the \f$ ax+by+c=0 \f$ equation of the epipolar line in the normalized plane of camera 2.

Let \f$ o + \lambda u \f$ be a point on a ray of origin \f$o\f$ and direction \f$u\f$.
Let \f$p\f$ be the intersection of this ray and the normalized plane \f$ z = 1 \f$.

Then \f$ \displaystyle p_x = \frac{o_x + \lambda u_x}{o_z + \lambda u_z} \f$ and \f$ \displaystyle p_y = \frac{o_y + \lambda u_y}{o_z + \lambda u_z} \f$,
which yields \f$ \displaystyle \lambda = -\frac{o_x - p_x o_z}{u_x - p_x u_z} \f$ and \f$ \displaystyle \lambda = -\frac{o_y - p_y o_z}{u_y - p_y u_z} \f$,
thus \f$ (o_x - p_x o_z) (u_y - p_y u_z) = (o_y - p_y o_z) (u_x - p_x u_z) \f$.

Finally, \f$ p_x (o_y u_z - o_z u_y) + p_y (o_z u_x - o_x u_z) + (o_x u_y - o_y u_x) = 0\f$,
which we can identify as \f$ ax+by+c=0 \f$.

Now it turns out that \f$ o \wedge u = (a,b,c) \f$,
so we use this last expression to simplify the code.

*/
LIBV_GEOMETRY_EXPORT Vector3d epipolar_line
( const Ray3d &ray ///< A ray originating from camera 1, as seen by camera 2.
)
{
  Vector3d e = ray.origin.cross(ray.direction);
  return e / e.head<2>().norm();
}

/**

Compute the distance between a point and an epipolar line.

\return
The distance between the point and the epipolar line, in the normalized plane.
This means you should multiply by the focal length if you want this distance in pixels.

*/
LIBV_GEOMETRY_EXPORT double epipolar_distance
( const UnifiedCameraModel &model ///< The model for this camera.
, const Eigen::Vector2d &pixel_in_image ///< A point seen by this camera.
, const Eigen::Vector3d &line ///< A line as returned by epipolar_line().
)
{
  // point in normalized plane
  Vector3d p;

  try
  {
    model.raytrace(pixel_in_image, p);
  }
  catch(...)
  {
    return std::numeric_limits<double>::infinity();
  }

  assert(p.z());
  p = p / p.z();

  // distance = ax+by+c = (a,b,c) . (x,y,1)
  return std::abs(line.transpose() * p);
}

/// \}
}}
