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

#include <iostream>
#include <stdexcept>

#include "camera_model.hpp"

using namespace Eigen;

namespace v {
namespace geometry {

/**

Initialize this model with all its parameters.

*/
UnifiedCameraModel::UnifiedCameraModel
( double fx ///< The focal length (x component).
, double fy ///< The focal length (y component).
, double u0 ///< The principal point (x component).
, double v0 ///< The principal point (y component).
, double xi ///< The distortion parameter.
)
: focal(fx, fy)
, center(u0, v0)
, xi(xi)
{
}

/**

Project a point from the camera coordinate system to the image coordinate system.

\return True if the projection is valid.

*/
bool UnifiedCameraModel::project
( const Eigen::Vector3d &point_in_camera ///< A point seen by the camera.
, Eigen::Vector2d &pixel_in_image ///< A pixel in the image.
) const
{
  // project the point on the sphere
  Vector3d p = point_in_camera.normalized();

  // bail out if the point is behind the camera
  if((xi > 1 && xi * p.z() <= -1) || (xi >= 0 && xi <= 1 && p.z() <= -xi)) return false;

  p.z() +=  xi;

  pixel_in_image = p.head<2>().array() * focal.array() / p.z() + center.array();
  return true;
}

/**

Trace a ray going through the camera and a pixel.

Let:
- \f$f\f$ be the focal length,
- \f$(u_0,v_0)\f$ the principal point,
- \f$(u,v)\f$ a pixel in the image,
- \f$m\f$ the coordinates of \f$(u,v)\f$ in the camera basis (on the normalized plane).

Then \f$ \displaystyle m = \left(\begin{array}{c} \frac{u - u_0}{f} \\ \frac{v - v_0}{f} \\ 1 \end{array}\right) \f$.

Also, let:
- \f$\xi\f$ be the distortion parameter,
- \f$\Xi\f$ the point \f$(0,0,\xi)\f$,
- \f$\mathcal{S}\f$ the unit sphere centered at \f$\Xi\f$,
- \f$\mathcal{L}\f$ a line going through the camera and \f$m\f$,
- \f$\eta\f$ a real number such that \f$ \eta m \in \mathcal{L} \cap \mathcal{S} \f$.

Then, \f$ \| \eta m - \Xi \| = 1 \f$, so \f$ (\| m \|^2) \eta^2 - (2 \xi) \eta + (\xi^2 - 1) = 0 \f$, so \f$ \displaystyle \eta = \frac{\xi + \sqrt{\xi^2 - (\xi^2 - 1) \| m \|^2}}{\| m \|^2} \f$.

Finally, let \f$a\f$ be the squared norm of \f$m\f$ seen as a 2D point (that is, \f$ a = m_x^2 + m_y^2 \f$).

Then \f$ \| m \|^2 = 1 + a \f$, so \f$ \displaystyle \eta = \frac{\xi + \sqrt{1 + a (1 - \xi^2)}}{1 + a} \f$.

The ray we are looking for is \f$\left(\begin{array}{c} \eta m_x \\ \eta m_y \\ \eta - \xi \end{array}\right)\f$.

*/
void UnifiedCameraModel::raytrace
( const Eigen::Vector2d &pixel_in_image ///< A pixel in the image.
, Eigen::Vector3d &ray_in_camera ///< A ray originating from the camera.
) const
{
  Vector2d p = (pixel_in_image - center).array() / focal.array();
  double a = p.squaredNorm();
  double b = 1 + a * (1 - xi * xi);
  if(b < 0) throw std::domain_error("the point is outside the image");
  double eta = (xi + std::sqrt(b)) / (1 + a);
  ray_in_camera << eta * p.x(), eta * p.y(), eta - xi;
}

/**

Formated output.

*/
LIBV_GEOMETRY_EXPORT std::ostream &operator<<(std::ostream &o, const UnifiedCameraModel &ucm)
{
  return o << "[[" << ucm.focal.transpose() << "] [" << ucm.center.transpose() << "] " << ucm.xi << "]";
}

}}
