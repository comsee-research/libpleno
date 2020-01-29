#pragma once

#include <Eigen/Geometry>

#include <libv/core/serialization/eigen.hpp>
#include <libv/core/serialization/serializable_properties.hpp>

template<std::size_t N> class Ray_;
/**

A pose in a N-D space.

*/
template<std::size_t N>
class Pose_ : public v::Serializable 
{
public:
	using Matrix =  Eigen::Matrix<double, N, N>;
	using Vector =	Eigen::Matrix<double, N, 1>;
	using Point  = 	Vector; 

private:
	Matrix rotation_;
	Vector translation_;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	Pose_(	const Matrix& rot = Matrix::Identity(),
		   	const Vector& trans = Vector::Zero()
		 );
	virtual ~Pose_();	   	

	Vector & translation();
	const Vector & translation() const;

	Matrix & rotation();
	const Matrix & rotation() const;

	virtual void serialize(v::InputArchive &) override;
	virtual void serialize(v::OutputArchive &) const override;
};

/**
 * returns a rotation matrix given 3 angles (radian) according to lie algebra and Taylor expansions
**/
Pose_<3>::Matrix rotation(const double alpha, const double beta, const double gamma);

template<std::size_t N>
inline std::ostream& operator<<(std::ostream& o, const Pose_<N>& p)
{
    o << "\ttranslation = {" <<  p.translation().transpose() << "};\n";
    o << "\trotation = {\n" << p.rotation() << "\n};\n";

    return o;
}


// reference: A frame of reference.
// point: A point expressed in the current frame of reference
//        (the frame where <tt>ref</> is expressed).
template<std::size_t N>
typename Pose_<N>::Point 
to_coordinate_system_of(const Pose_<N>& reference, const typename Pose_<N>::Point& point)
{
    return reference.rotation().transpose() * (point - reference.translation());
}

// reference: A frame of reference.
// point: A point expressed in the world coordinate system
template<std::size_t N>
typename Pose_<N>::Point 
from_coordinate_system_of(const Pose_<N>& reference, const typename Pose_<N>::Point& point)
{
    return (reference.rotation() * point) + reference.translation();
}

// reference: A frame of reference.
// pose: A pose expressed in the given frame of reference <tt>ref</tt>.
template<std::size_t N>
Pose_<N> to_coordinate_system_of(const Pose_<N>& reference, const Pose_<N>& pose)
{
    Pose_<N> p;
    p.translation() = to_coordinate_system_of(reference, pose.translation());
    p.rotation() = reference.rotation().transpose() * pose.rotation();

    return p;
};

// reference: A frame of reference.
// pose: A pose expressed in the world coordinate system
template<std::size_t N>
Pose_<N> from_coordinate_system_of(const Pose_<N>& reference, const Pose_<N>& pose)
{
    Pose_<N> p;
    p.translation() = from_coordinate_system_of(reference, pose.translation());
    p.rotation() = reference.rotation() * pose.rotation();

    return p;
}

/**
 * Transform a ray from the current reference frame to another reference frame.
 * In other words, given <tt>r</tt> and a pose <tt>p</tt> expressed in the same (current) frame,
 * it returns an equivalent ray expressed in the <tt>p</tt> frame.
 *
 * reference: A frame of reference.
 * ray: A ray expressed in the world coordinate system
**/
template<std::size_t N>
Ray_<N> to_coordinate_system_of(const Pose_<N>& reference, const Ray_<N>& ray)
{
    return Ray_<N>{to_coordinate_system_of(reference, ray.origin()),
                 reference.rotation().transpose() * ray.direction(),
                 ray.color()};
}

/**
 * Transform a ray from the given reference frame to the current reference frame.
 * In other words, given <tt>r</tt> expressed in the <tt>p</tt> frame,
 * it returns an equivalent ray expressed in the frame where <tt>p</tt> is expressed.
 *
 * reference: A frame of reference.
 * ray: A ray expressed in the current frame of reference
**/
template<std::size_t N> 
Ray_<N> from_coordinate_system_of(const Pose_<N>& reference, const Ray_<N> &ray)
{
    return Ray_<N>{from_coordinate_system_of(reference, ray.origin()),
                 reference.rotation() * ray.direction(),
                 ray.color()};
};


using Pose = Pose_<3ul>;
using Pose3D = Pose_<3ul>;
using Pose2D = Pose_<2ul>;
