#include "ray.h"

// Constructor
template<std::size_t N>
Ray_<N>::Ray_(
	const typename Ray_<N>::Vector & o, 
	const typename Ray_<N>::Vector & d, 
	const RGBA& c
) : origin_(o), direction_(d), color_(c)
{
 	if( direction_ == Ray_<N>::Vector::Zero())
 		direction_[N-1] = 1.;
 		
 	direction_.normalize();
}

template<std::size_t N> Ray_<N>::~Ray_() {}

template<std::size_t N>
typename Ray_<N>::Vector & Ray_<N>::origin() { return origin_; }
template<std::size_t N>
const typename Ray_<N>::Vector & Ray_<N>::origin() const { return origin_; }

template<std::size_t N>
typename Ray_<N>::Vector & Ray_<N>::direction() { return direction_; }
template<std::size_t N>
const typename Ray_<N>::Vector & Ray_<N>::direction() const { return direction_; }

template<std::size_t N> RGBA& Ray_<N>::color() { return color_; }
template<std::size_t N> const RGBA& Ray_<N>::color() const { return color_; }

// Configure a ray according using 2 points
// p1 is new origin
template<std::size_t N>
void Ray_<N>::config(const typename Ray_<N>::Vector& p1,
                   const typename Ray_<N>::Vector& p2,
                   const RGBA& c)
{
    origin_ = p1;
    direction_ = (p2 - p1).normalized();
    color_ = c;
}

// Return a point on a ray
template<std::size_t N>
typename Ray_<N>::Vector Ray_<N>::operator()(const double t) const
{
    return (direction_ * t + origin_);
}

template class Ray_<2ul>;
template class Ray_<3ul>;

//------------------------------------------------------------------------------
// Determine the t coefficient of a parametric line equation where its cross a plane
double compute_t_coef(const Eigen::Matrix<double, 4, 1>& plane, const Ray3D& line)
{
    double t = -( plane(0) * line.origin()(0)
                + plane(1) * line.origin()(1)
                + plane(2) * line.origin()(2)
                + plane(3) );

    return t / (plane(0) * line.direction()(0)
              + plane(1) * line.direction()(1)
              + plane(2) * line.direction()(2));
}

// Determine the intersection point between a plane and a line
P3D line_plane_intersection(const Eigen::Matrix<double, 4, 1>& plane, const Ray3D& ray)
{
    //TODO
    // if (RAYON !// AU PLAN)
    // {
        const double t = compute_t_coef(plane, ray);
        return ray(t);
    // }
}

// return the ditance between a point to a line
double line_point_distance(const Ray3D& ray, const P3D& point)
{
    P3D out = ray.origin() + ray.direction() * ray.direction().dot(point - ray.origin()) / ray.direction().squaredNorm();

    return (out - point).norm();
}
