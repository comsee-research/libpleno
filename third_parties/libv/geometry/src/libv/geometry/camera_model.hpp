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

#ifndef LIBV_GEOMETRY_CAMERA_MODEL_HPP
#define LIBV_GEOMETRY_CAMERA_MODEL_HPP

#include <Eigen/Core>

#include "global.hpp"
#include <boost/serialization/access.hpp>

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

/**

The unified camera model.

*/
struct LIBV_GEOMETRY_EXPORT UnifiedCameraModel
{
  friend class boost::serialization::access;
  UnifiedCameraModel(double = 1, double = 1, double = 0, double = 0, double = 0);
  bool project(const Eigen::Vector3d &, Eigen::Vector2d &) const;
  void raytrace(const Eigen::Vector2d &, Eigen::Vector3d &) const;

  /// The focal length.
  Eigen::Vector2d focal;

  /// The principal point.
  Eigen::Vector2d center;

  /// The distortion parameter.
  double xi;
};

/// \}
}}

#endif
