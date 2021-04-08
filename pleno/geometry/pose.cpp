#include "pose.h"

#include "geometry/ray.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
template<std::size_t N>
Pose_<N>::Pose_(const Rotation_<N>& rot, const Translation_<N>& trans)
: rotation_(rot), translation_(trans)
{}

template<std::size_t N>
Pose_<N>::~Pose_()
{}

template<std::size_t N>
Translation_<N>& Pose_<N>::translation() { return translation_; }
template<std::size_t N>
const Translation_<N>& Pose_<N>::translation() const { return translation_; }

template<std::size_t N>
Rotation_<N> & Pose_<N>::rotation() { return rotation_; }
template<std::size_t N>
const Rotation_<N> & Pose_<N>::rotation() const { return rotation_; }

template<std::size_t N>
void Pose_<N>::serialize(v::InputArchive &archive)
{
	archive
	("rotation", rotation_)
	("translation", translation_)
	;
	//rotation(rotation_orthogonalize(rotation()));
}
template<std::size_t N>
void Pose_<N>::serialize(v::OutputArchive &archive) const
{
	archive
	("rotation", rotation_)
	("translation", translation_)
	;
}

template class Pose_<2ul>;
template class Pose_<3ul>;


template<std::size_t N>
std::ostream& operator<<(std::ostream& o, const Pose_<N>& p)
{
    o << "\ttranslation = {" <<  p.translation().transpose() << "},\n";
    o << "\trotation = {\n" << p.rotation() << "\n};\n";

    return o;
}

template<>
std::ostream& operator<<(std::ostream& o, const Pose_<3ul>& p)
{
    o << "\ttranslation = {" <<  p.translation().transpose() << "},\n";
    o << "\trotation = {\n" << p.rotation() << "\n},\n";
    o << "\tRxyz = {" << p.rotation().eulerAngles(0,1,2).transpose() << "};\n";

    return o;
}

template<>
std::ostream& operator<<(std::ostream& o, const Pose_<2ul>& p)
{
    o << "\ttranslation = {" <<  p.translation().transpose() << "},\n";
    o << "\trotation = {\n" << p.rotation() << "\n},\n";
    o << "\ttheta = " << std::atan2(p.rotation()(1, 0), p.rotation()(0, 0)) << "\n";

    return o;
}

// reference: A frame of reference.
// point: A point expressed in the current frame of reference
//        (the frame where <tt>ref</> is expressed).
template<std::size_t N>
PnD<N> to_coordinate_system_of(const Pose_<N>& reference, const PnD<N>& point)
{
    return reference.rotation().transpose() * (point - reference.translation());
}

template PnD<3ul> to_coordinate_system_of(const Pose_<3ul>& reference, const PnD<3ul>& point);
template PnD<2ul> to_coordinate_system_of(const Pose_<2ul>& reference, const PnD<2ul>& point);

// reference: A frame of reference.
// point: A point expressed in the world coordinate system
template<std::size_t N>
PnD<N> from_coordinate_system_of(const Pose_<N>& reference, const PnD<N>& point)
{
    return (reference.rotation() * point) + reference.translation();
}

template PnD<3ul> from_coordinate_system_of(const Pose_<3ul>& reference, const PnD<3ul>& point);
template PnD<2ul> from_coordinate_system_of(const Pose_<2ul>& reference, const PnD<2ul>& point);

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
template Pose_<3ul> to_coordinate_system_of(const Pose_<3ul>& reference, const Pose_<3ul>& pose);
template Pose_<2ul> to_coordinate_system_of(const Pose_<2ul>& reference, const Pose_<2ul>& pose);

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
template Pose_<3ul> from_coordinate_system_of(const Pose_<3ul>& reference, const Pose_<3ul>& pose);
template Pose_<2ul> from_coordinate_system_of(const Pose_<2ul>& reference, const Pose_<2ul>& pose);

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

template Ray_<3ul> to_coordinate_system_of(const Pose_<3ul>& reference, const Ray_<3ul>& ray);
template Ray_<2ul> to_coordinate_system_of(const Pose_<2ul>& reference, const Ray_<2ul>& ray);

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

template Ray_<3ul> from_coordinate_system_of(const Pose_<3ul>& reference, const Ray_<3ul>& ray);
template Ray_<2ul> from_coordinate_system_of(const Pose_<2ul>& reference, const Ray_<2ul>& ray);
