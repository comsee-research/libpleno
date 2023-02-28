/**

\file
Simple class for a ray in 3D space.
\author Pierre Lébraly (2012)
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

#ifndef LIBV_GEOMETRY_RAY_HPP
#define LIBV_GEOMETRY_RAY_HPP

#include <Eigen/Geometry>
#include <limits>

#include "global.hpp"

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/// A Ray is a 3D-line defined by a 3D-point and a unit direction vector
/// \implements std::EqualityComparable
template<class Scalar, int dimension>
struct Ray
{
  /// The type of vector.
  typedef Eigen::Matrix<Scalar, dimension, 1> Vector;

  /// The origin of this ray.
  Vector origin;

  /// The direction of this ray.
  Vector direction;

  /**

  Constructor.

  */
  Ray
  ( const Vector &direction = Vector::UnitX()
  , const Vector &origin = Vector::Zero()
  )
  : origin(origin)
  , direction(direction.normalized())
  {
  }

  /**

  \returns True if \e this and \e other generate the same oriented line.

  */
  template<class X> bool
  operator==
  ( const X &other ///< Another ray.
  ) const
  {
    return origin == other.origin && direction == other.direction;
  }

  /**

  \returns False if \e this and \e other generate the same oriented line.

  */
  template<class X> bool
  operator!=
  ( const X &other ///< Another ray.
  ) const
  {
    return !(*this == other);
  }

  /**

  Rescale the ray (uniform scaling applied to the coordinates system of the 3D-point).

  \returns A rescaled ray.

  */
  Ray operator*
  ( Scalar scale ///< scale factor to be applied
  ) const
  {
    return Ray(direction, origin * scale);
  }
};

/// \copydoc operator==()
template<> template<>
inline bool Ray<float, 3>::operator==
( const Ray<float, 3> &other ///< An other ray.
) const
{
  return ((origin - other.origin).array().abs() < 10 * std::numeric_limits<float>::epsilon()).all() && ((direction - other.direction).array().abs() < 10 * std::numeric_limits<float>::epsilon()).all();
}

/// A ray is an object of space on which can be tranformed (by a rotation, a translation,
/// an affine transformation, a projective transformation,...)
template<class Scalar, int dimension, int mode, int options>
Ray<Scalar, dimension> operator*(const Eigen::Transform<Scalar, dimension, mode, options> &transform, const Ray<Scalar, dimension> &ray)
{
  return Ray<Scalar, dimension>(transform.linear() * ray.direction, transform * ray.origin);
}

/// Formatted output.
template<class Scalar, int dimension>
std::ostream &operator<<(std::ostream &o, const Ray<Scalar, dimension> &ray)
{
  return o << "(" << ray.origin.transpose() << ") + k * (" << ray.direction.transpose() << ")";
}

/// A 3-dimensional ray.
typedef Ray<float, 3>  Ray3f;

/// A 3-dimensional ray.
typedef Ray<double, 3> Ray3d;

/// \}
}}

#endif
