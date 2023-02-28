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

#include "pose.hpp"
#include "rotation.hpp"

namespace v {
namespace geometry {

Pose::Pose():
  translation_(Eigen::Vector3d(0, 0, 0)),
  rotation_(Eigen::Matrix3d::Identity())
{
}

Pose::Pose(const Eigen::Isometry3d &other)
{
  translation_ = other.translation();
  rotation_ = other.rotation();
}

Pose::operator Eigen::Isometry3d() const
{
  return Eigen::Isometry3d(Eigen::Isometry3d::Identity())
    .translate(translation_)
    .rotate(rotation_);
}

void Pose::serialize(v::InputArchive &archive)
{
  archive
  ("rotation", rotation_)
  ("translation", translation_)
  ;
  rotation(rotation_orthogonalize(rotation()));
}

void Pose::serialize(v::OutputArchive &archive) const
{
  archive
  ("rotation", rotation_)
  ("translation", translation_)
  ;
}

Pose &Pose::rotation_axis_angle(const Eigen::Vector3d &axis_angle)
{
  rotation(Eigen::Matrix3d::Identity());
  apply_rotation(rotation(), axis_angle);
  return *this;
}

/**

Transform a pose from the current reference frame to another reference frame.
In other words, given two poses <tt>ref</tt> and <tt>pose</tt> expressed in the same (current) frame, it returns an equivalent pose expressed in the <tt>ref</tt> frame.

\return
The pose expressed in the given frame of reference <tt>ref</tt>.

*/
LIBV_GEOMETRY_EXPORT Pose to_coordinate_system_of
( const Pose &ref ///< A frame of reference.
, const Pose &pose ///< A pose expressed in the current frame of reference (the frame where <tt>ref</> is expressed).
)
{
  return Pose().rotation(ref.rotation().transpose() * pose.rotation()).translation(to_coordinate_system_of(ref, pose.translation()));
}

/**

Transform a pose from the given reference frame to the current reference frame.
In other words, given <tt>pose</tt> expressed in the <tt>ref</tt> frame, it returns an equivalent pose expressed in the frame where <tt>ref</tt> is expressed.

\return
The pose expressed in the current frame of reference (the frame where <tt>ref</> is expressed).

*/
LIBV_GEOMETRY_EXPORT Pose from_coordinate_system_of
( const Pose &ref ///< A frame of reference.
, const Pose &pose ///< A pose expressed in the given frame of reference <tt>ref</tt>.
)
{
  return Pose().rotation(ref.rotation() * pose.rotation()).translation(from_coordinate_system_of(ref, pose.translation()));
}

/**

Transform a point from the current reference frame to another reference frame.
In other words, given <tt>point</tt> and a pose <tt>ref</tt> expressed in the same (current) frame, it returns an equivalent point expressed in the <tt>ref</tt> frame.

\return
The point expressed in the given frame of reference <tt>ref</tt>.

*/
LIBV_GEOMETRY_EXPORT Eigen::Vector3d to_coordinate_system_of
( const Pose &ref ///< A frame of reference.
, const Eigen::Vector3d &point ///< A point expressed in the current frame of reference (the frame where <tt>ref</> is expressed).
)
{
  return ref.rotation().transpose() * (point - ref.translation());
}

/**

Transform a point from the given reference frame to the current reference frame.
In other words, given <tt>point</tt> expressed in the <tt>ref</tt> frame, it returns an equivalent point expressed in the frame where <tt>ref</tt> is expressed.

\return
The point expressed in the current frame of reference (the frame where <tt>ref</> is expressed).

*/
LIBV_GEOMETRY_EXPORT Eigen::Vector3d from_coordinate_system_of
( const Pose &ref ///< A frame of reference.
, const Eigen::Vector3d &point ///< A point expressed in the given frame of reference <tt>ref</tt>.
)
{
  return (ref.rotation() * point) + ref.translation();
}

/**

Transform a ray from the current reference frame to another reference frame.
In other words, given <tt>ray</tt> and a pose <tt>ref</tt> expressed in the same (current) frame, it returns an equivalent ray expressed in the <tt>ref</tt> frame.

\return
The ray expressed in the given frame of reference <tt>ref</tt>.

*/
LIBV_GEOMETRY_EXPORT Ray3d to_coordinate_system_of
( const Pose &ref ///< A frame of reference.
, const Ray3d &ray ///< A ray expressed in the current frame of reference (the frame where <tt>ref</> is expressed).
)
{
  return Ray3d(ref.rotation().transpose() * ray.direction, to_coordinate_system_of(ref, ray.origin));
}

/**

Transform a ray from the given reference frame to the current reference frame.
In other words, given <tt>ray</tt> expressed in the <tt>ref</tt> frame, it returns an equivalent ray expressed in the frame where <tt>ref</tt> is expressed.

\return
The ray expressed in the current frame of reference (the frame where <tt>ref</> is expressed).

*/
LIBV_GEOMETRY_EXPORT Ray3d from_coordinate_system_of
( const Pose &ref ///< A frame of reference.
, const Ray3d &ray ///< A ray expressed in the given frame of reference <tt>ref</tt>.
)
{
  return Ray3d(ref.rotation() * ray.direction, from_coordinate_system_of(ref, ray.origin));
}

}
}
