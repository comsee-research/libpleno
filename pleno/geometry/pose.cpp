#include "pose.h"

#include "geometry/ray.h"

// returns a rotation matrix given 3 angles (radian) according to lie algebra and Taylor expansions
Pose_<3>::Matrix rotation(const double a, const double b, const double g)
{
    double theta2 = a * a + b * b + g * g + std::numeric_limits<double>::epsilon();

    double theta = std::sqrt(theta2);

    const double factor = std::sin(theta) / theta2;
    const double t = std::tan(0.5 * theta);

    Pose_<3>::Matrix matrix;
    matrix <<
    -(b * b + g * g)  *  t, a * b * t - g * theta, a * g * t + b * theta,
     a * b * t + g * theta,  -(a * a + g * g) * t, b * g * t - a * theta,
     a * g * t - b * theta, b * g * t + a * theta,  -(a * a + b * b) * t;

    matrix *= factor;
    matrix += Pose_<3>::Matrix::Identity();

    return matrix;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template<std::size_t N>
Pose_<N>::Pose_(const typename Pose_<N>::Matrix& rot, const typename Pose_<N>::Vector& trans)
: _rotation(rot), _translation(trans)
{}

template<std::size_t N>
Pose_<N>::~Pose_()
{}

template<std::size_t N>
typename Pose_<N>::Vector & Pose_<N>::translation() { return _translation; }
template<std::size_t N>
const typename Pose_<N>::Vector & Pose_<N>::translation() const { return _translation; }

template<std::size_t N>
typename Pose_<N>::Matrix & Pose_<N>::rotation() { return _rotation; }
template<std::size_t N>
const typename Pose_<N>::Matrix & Pose_<N>::rotation() const { return _rotation; }



template<std::size_t N>
void Pose_<N>::serialize(v::InputArchive &archive)
{
	archive
	("rotation", _rotation)
	("translation", _translation)
	;
	//rotation(rotation_orthogonalize(rotation()));
}
template<std::size_t N>
void Pose_<N>::serialize(v::OutputArchive &archive) const
{
	archive
	("rotation", _rotation)
	("translation", _translation)
	;
}

template class Pose_<2ul>;
template class Pose_<3ul>;
