#include "ray.h"

#include "io/printer.h"

// Constructor
template<std::size_t N>
Ray_<N>::Ray_(
	const PnD<N>& o, 
	const PnD<N>& d, 
	const RGBA& c
) : origin_(o), direction_(d), color_(c)
{
 	if( direction_ == PnD<N>::Zero())
 		direction_[N-1] = 1.;
 		
 	direction_.normalize();
}

// Copy Constructor
template<std::size_t N>
Ray_<N>::Ray_(
	const Ray_<N>& o
) : origin_(o.origin()), direction_(o.direction()), color_(o.color())
{
}

// Move Constructor
template<std::size_t N>
Ray_<N>::Ray_(
	Ray_<N>&& o
) : origin_(std::move(o.origin_)), direction_(std::move(o.direction_)), color_(std::move(o.color_))
{
}

template<std::size_t N> Ray_<N>::~Ray_() {}

template<std::size_t N>
PnD<N>& Ray_<N>::origin() { return origin_; }
template<std::size_t N>
const PnD<N>& Ray_<N>::origin() const { return origin_; }

template<std::size_t N>
PnD<N>& Ray_<N>::direction() { return direction_; }
template<std::size_t N>
const PnD<N>& Ray_<N>::direction() const { return direction_; }

template<std::size_t N> RGBA& Ray_<N>::color() { return color_; }
template<std::size_t N> const RGBA& Ray_<N>::color() const { return color_; }

// Configure a ray according using 2 points
// p1 is new origin
template<std::size_t N>
void Ray_<N>::config(const PnD<N>& p1,
                   const PnD<N>& p2,
                   const RGBA& c)
{
    DEBUG_ASSERT((p1 != p2),"Ray::config: p1 == p2, no direction can be computed");
    origin_ = p1;
    direction_ = (p2 - p1).normalized();
    color_ = c;
}

// Return a point on a ray
template<std::size_t N>
PnD<N> Ray_<N>::operator()(double t) const
{
    return (direction_ * t + origin_);
}

template class Ray_<2ul>;
template class Ray_<3ul>;

//------------------------------------------------------------------------------
// Determine the t coefficient of a parametric line equation where its cross a plane
double compute_t_coef(const PlaneCoefficients& plane, const Ray3D& line)
{                
	DEBUG_ASSERT(
		(line.direction()(0) != 0.0 and line.direction()(1) != 0.0 and line.direction()(2) != 0.0),
		"In compute_t_coef: Direction is null (division by 0)"	
	);
	
    double t = -( plane(0) * line.origin()(0)
                + plane(1) * line.origin()(1)
                + plane(2) * line.origin()(2)
                + plane(3) );
	
    return t / (plane(0) * line.direction()(0)
              + plane(1) * line.direction()(1)
              + plane(2) * line.direction()(2));
}

// Determine the intersection point between a plane and a line
P3D line_plane_intersection(const PlaneCoefficients& plane, const Ray3D& ray)
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
