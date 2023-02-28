/**

\file
Des fonctions pour convertir les types ROS en types Libv.

Ces fonctions étaient initialement définies dans la bibliothèque libv_ros écrite par Gérald Lelong.

\author Gérald Lelong (2018)

*/

#ifndef LIBV_GEOMETRY_ROS_HPP
#define LIBV_GEOMETRY_ROS_HPP

#include <libv/geometry/found/eigen_conversions>
#include <libv/geometry/found/geometry_msgs>
#if !defined LIBV_IGNORE_OPTIONAL_DEPENDENCIES ||\
  defined LIBV_EIGEN_CONVERSIONS_FOUND &&\
  defined LIBV_GEOMETRY_GEOMETRY_MSGS_FOUND &&\
  1
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf_conversions/tf_eigen.h>

#include <libv/geometry/pose.hpp>

namespace v {
namespace geometry {
/// \addtogroup geometry
/// \{

inline Pose make_pose(const geometry_msgs::Pose &msg)
{
  Eigen::Isometry3d e;
  tf::poseMsgToEigen(msg, e); /// requires eigen_conversions
  return Pose().translation(e.translation()).rotation(e.rotation());
}

inline Pose make_pose(const geometry_msgs::PoseStamped &msg)
{
  return make_pose(msg.pose);
}

inline Pose make_pose(const geometry_msgs::PoseWithCovariance &msg)
{
  return make_pose(msg.pose);
}

inline Pose make_pose(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  return make_pose(msg.pose);
}

inline Pose make_pose(const tf::Transform &transform)
{
  Eigen::Isometry3d e;
  tf::transformTFToEigen(transform, e); /// requires tf_conversions
  return Pose().translation(e.translation()).rotation(e.rotation());
}

/// \}
}
}

#endif
#endif
