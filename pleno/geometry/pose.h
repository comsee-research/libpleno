#pragma once

#include <Eigen/Geometry>

#include "types.h"

#include "processing/tools/error.h"
#include "io/archive.h"

template<std::size_t N> class Ray_;
template<std::size_t N> class Pose_;

/**
	A pose in a N-D space.
**/
template<std::size_t N>
class Pose_ : public v::Serializable 
{
	Rotation_<N> rotation_;
	Translation_<N> translation_;
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Pose_(const Rotation_<N>& rot = Rotation_<N>::Identity(), const Translation_<N>& trans = Translation_<N>::Zero());
	virtual ~Pose_();	   	

	Translation_<N>& translation();
	const Translation_<N>& translation() const;

	Rotation_<N>& rotation();
	const Rotation_<N>& rotation() const;

	void serialize(v::InputArchive &) override;
	void serialize(v::OutputArchive &) const override;
};

struct CalibrationPose 	{ Pose_<3ul> pose; Index frame; };
struct PoseWithError 	{ Pose_<3ul> pose; RMSE rmse; };

using Pose = Pose_<3ul>;
using Pose3D = Pose_<3ul>;
using Pose2D = Pose_<2ul>;

template<std::size_t N>
std::ostream& operator<<(std::ostream& o, const Pose_<N>& p);

// reference: A frame of reference.
// point: A point expressed in the current frame of reference
//        (the frame where <tt>ref</> is expressed).
template<std::size_t N>
PnD<N> to_coordinate_system_of(const Pose_<N>& reference, const PnD<N>& point);

// reference: A frame of reference.
// point: A point expressed in the world coordinate system
template<std::size_t N>
PnD<N> from_coordinate_system_of(const Pose_<N>& reference, const PnD<N>& point);

// reference: A frame of reference.
// pose: A pose expressed in the given frame of reference <tt>ref</tt>.
template<std::size_t N>
Pose_<N> to_coordinate_system_of(const Pose_<N>& reference, const Pose_<N>& pose);

// reference: A frame of reference.
// pose: A pose expressed in the world coordinate system
template<std::size_t N>
Pose_<N> from_coordinate_system_of(const Pose_<N>& reference, const Pose_<N>& pose);

/**
 * Transform a ray from the current reference frame to another reference frame.
 * In other words, given <tt>r</tt> and a pose <tt>p</tt> expressed in the same (current) frame,
 * it returns an equivalent ray expressed in the <tt>p</tt> frame.
 *
 * reference: A frame of reference.
 * ray: A ray expressed in the world coordinate system
**/
template<std::size_t N>
Ray_<N> to_coordinate_system_of(const Pose_<N>& reference, const Ray_<N>& ray);

/**
 * Transform a ray from the given reference frame to the current reference frame.
 * In other words, given <tt>r</tt> expressed in the <tt>p</tt> frame,
 * it returns an equivalent ray expressed in the frame where <tt>p</tt> is expressed.
 *
 * reference: A frame of reference.
 * ray: A ray expressed in the current frame of reference
**/
template<std::size_t N> 
Ray_<N> from_coordinate_system_of(const Pose_<N>& reference, const Ray_<N> &ray);
