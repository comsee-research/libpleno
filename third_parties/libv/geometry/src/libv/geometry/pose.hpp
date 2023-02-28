/**

\file
\author Alexis Wilhelm (2014)
\copyright 2014 Institut Pascal

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

#ifndef LIBV_GEOMETRY_POSE_HPP
#define LIBV_GEOMETRY_POSE_HPP

#include <libv/core/serialization/eigen.hpp>
#include <libv/core/serialization/serializable_properties.hpp>
#include <boost/serialization/access.hpp>

#include "ray.hpp"

namespace v {
namespace geometry {

/// \addtogroup geometry
/// \{

/**

A pose in a 3D space.

*/
struct LIBV_GEOMETRY_EXPORT Pose: public v::Serializable
{
  Pose();
  Pose(const Eigen::Isometry3d &);
  operator Eigen::Isometry3d() const;

  Eigen::Vector3d & translation() { return translation_; }
  const Eigen::Vector3d & translation() const { return translation_; }
  Pose & translation(const Eigen::Vector3d &t) { translation_ = t; return *this; }

  Eigen::Matrix3d & rotation() { return rotation_; }
  const Eigen::Matrix3d & rotation() const { return rotation_; }
  Pose & rotation(const Eigen::Matrix3d &r) { rotation_ = r; return *this; }

  virtual void serialize(v::InputArchive &) override;
  virtual void serialize(v::OutputArchive &) const override;

  Pose &rotation_axis_angle(const Eigen::Vector3d &);

private:
  Eigen::Vector3d translation_;
  Eigen::Matrix3d rotation_;
};

LIBV_GEOMETRY_EXPORT Pose to_coordinate_system_of(const Pose &, const Pose &);
LIBV_GEOMETRY_EXPORT Pose from_coordinate_system_of(const Pose &, const Pose &);
LIBV_GEOMETRY_EXPORT Eigen::Vector3d to_coordinate_system_of(const Pose &, const Eigen::Vector3d &);
LIBV_GEOMETRY_EXPORT Eigen::Vector3d from_coordinate_system_of(const Pose &, const Eigen::Vector3d &);
LIBV_GEOMETRY_EXPORT Ray3d to_coordinate_system_of(const Pose &, const Ray3d &);
LIBV_GEOMETRY_EXPORT Ray3d from_coordinate_system_of(const Pose &, const Ray3d &);

/// \}
}
}

#endif
