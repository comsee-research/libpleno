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

#include "rotation.hpp"

#include <cmath>

#include <boost/math/special_functions/sinc.hpp>

#include <Eigen/SVD>
#include <Eigen/LU>

using namespace Eigen;

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/**

Apply a rotation to a transformation.

As \f$ d = (x,y,z) \f$ is a rotation in axis-angle representation, its logarithm map is :

\f[ \left(\begin{array}{ccc} 0 & -z & y \\ z & 0 & -x \\ -y & x & 0 \end{array}\right) \f]

then, the rotation is applied with:
\f[ m \leftarrow m \times \exp \left(\begin{array}{ccc} 0 & -z & y \\ z & 0 & -x \\ -y & x & 0 \end{array}\right) \f]

\warning This function could be unstable for a rotation with angle near from π (in practice if \f$ \theta \in [\pi-10^{-4}, \pi+10^{-4}] \f$).

*/
LIBV_GEOMETRY_EXPORT void apply_rotation
( Eigen::Matrix3d &m ///< A transformation matrix.
, const Eigen::Vector3d &d ///< A rotation in axis-angle representation. This vector is the axis of rotation and his norm is the angle of rotation.
)
{
  Matrix3d skrew;
  skrew << 0, -d.z(), d.y(), d.z(), 0, -d.x(), -d.y(), d.x(), 0;
  m *= rotation_exp(skrew);
}

/**

Apply a small rotation to a transformation.
This is a first order approximation, useful when computing a numerical derivative.
\f[ m \leftarrow m \times \left(\begin{array}{ccc} 1 & -z & y \\ z & 1 & -x \\ -y & x & 1 \end{array}\right) \f]

*/
LIBV_GEOMETRY_EXPORT void apply_small_rotation
( Eigen::Matrix3d &m ///< A transformation matrix.
, const Eigen::Vector3d &d ///< A small rotation in axis-angle representation.
)
{
  Matrix3d skrew;
  skrew << 1, -d.z(), d.y(), d.z(), 1, -d.x(), -d.y(), d.x(), 1;
  m *= skrew;
}

/**

The distance between two rotations.

It uses the Rodrigues formula apply with the SO(3) logarithm map : \f$ || log(R) || = \sqrt{2} |\theta| \f$ with: \f$ \theta = \arccos \frac{\mathrm{trace}(R)-1}{2} \f$.

This formula is used because the geodesic distance between two rotation matrices (\f$ A \f$ and \f$ B \f$) could be defined with: \f$ || \log(A B^\top) || \f$.

The return value is then \f$\sqrt{2} |\theta|\f$ where \f$ \theta \f$ is the angle of the rotation between \f$ A \f$ and \f$ B \f$.

\return The a distance between \e a and \e b.

*/
LIBV_GEOMETRY_EXPORT double geodesic_distance
( const Eigen::Matrix3d &a ///< A rotation matrix.
, const Eigen::Matrix3d &b ///< A rotation matrix.
)
{
  // Rodrigues formula is faster than with matrix log
  // very sensible to non-orthogonal matrices
  double r = .5 * ((a*b.transpose()).trace()-1.);
  if( r > 1 )
   return 0;
  if( r < -1 )
   return M_SQRT2 * M_PI;
  else
   return M_SQRT2 * std::fabs(std::acos(r));
}

/**
Exponential map of a skew-symmetric matrix to \f$ SO(3)\f$.

\param[in] a : a skew-symmetric matrix.
\return a rotation matrix that is the exponential map of \e a to \f$ SO(3)\f$.

The resulting rotation is calculated with:
\f[ e^R = Id + \frac{\sin{\theta}}{\theta}R + \frac{1-\cos{\theta}}{\theta^2}R^2 \f]

*/
LIBV_GEOMETRY_EXPORT Matrix3d rotation_exp(const Eigen::Matrix3d &a)
{

  double theta2 = a(0,1)*a(0,1) + a(0,2)*a(0,2) +  a(1,2)*a(1,2) + std::numeric_limits<double>::epsilon();
  double theta = std::sqrt(theta2);
  return Matrix3d::Identity() + boost::math::sinc_pi(theta)*a+(1.0-std::cos(theta))/theta2*a*a;
}

/**
Logarithm map of a rotation matrix to a skew-symmetric matrix.
\param[in] a : a rotation matrix.
\return a skew-symmetric matrix that is the logarithm map of \e a from \f$ SO(3) \f$.

The resulting skew-symmetric matrix is calultated with:

\f[ log(R) = \frac{\theta}{2\cos{\theta}}(R-R^\top) \f]

\warning This function could be unstable for a rotation with angle near from π (in practice if \f$ \theta \in [\pi-10^{-4}, \pi+10^{-4}] \f$).
*/
LIBV_GEOMETRY_EXPORT Matrix3d rotation_log(const Eigen::Matrix3d &a)
{
  double tmp = 0.5*(a.trace()-1.0);
// if need more precision for an angle near from pi, you can uncomment the following 2 lines
//  tmp = std::max(tmp,-1.0);
//  tmp = std::min(tmp,1.0-std::numeric_limits<double>::epsilon());
  return 0.5/boost::math::sinc_pi(std::acos(tmp))*(a-a.transpose());
}


/**

Linear interpolation of rotation matrices.

\return An interpolated rotation between \e a and \e b.

\warning This function could be unstable for a rotation with angle near from π (in practice if \f$ \theta \in [\pi-10^{-4}, \pi+10^{-4}] \f$).

*/
LIBV_GEOMETRY_EXPORT Matrix3d rotation_linear_interpolation
( const Eigen::Matrix3d &a ///< A rotation matrix.
, const Eigen::Matrix3d &b ///< Idem.
, double t ///< A parameter in [0,1].
)
{
  return a * rotation_exp(t * rotation_log(a.transpose() * b));
}

/** \brief A function to make 3x3 matrix a rotation matrix.
  This function is based on the SVD decomposition.
  \param m: input 3x3 matrix
  \return a rotation matrix (near from the input matrix)
*/
LIBV_GEOMETRY_EXPORT Eigen::Matrix3d rotation_orthogonalize(const Eigen::Matrix3d & m)
{

  // SVD
  Eigen::JacobiSVD<Eigen::Matrix3d> svd_solver;
  svd_solver.compute(m, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Build Rotation Matrix
  Eigen::Matrix3d r = svd_solver.matrixU() * svd_solver.matrixV().transpose();

  // If bad side we permute a basis vector
  if (r.determinant()<0)
  {
    Eigen::Matrix3d u = svd_solver.matrixU();
    u(0,2) = -u(0,2); u(1,2) = -u(1,2); u(2,2) = -u(2,2);
    r = u * svd_solver.matrixV().transpose();
  }

  return r;

}

/// \}
}}
