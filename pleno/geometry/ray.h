#pragma once //from charlib

#include "types.h"

#include "io/archive.h"

#include "geometry/plane.h"

template<std::size_t N>
class Ray_
{ 
    PnD<N> origin_;
    PnD<N> direction_;
    RGBA color_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    //Constructor
    Ray_(const PnD<N>& o = PnD<N>::Zero(),
         const PnD<N>& d = PnD<N>::Zero(),
         const RGBA& c = {0.0, 0.0, 0.0, 255.0});
         
    Ray_(const Ray_& o);
    Ray_(Ray_&& o);

    ~Ray_();
    
	PnD<N> & origin();
	const PnD<N> & origin() const;
	
	PnD<N> & direction();
	const PnD<N> & direction() const;
	
	RGBA & color();
	const RGBA & color() const;
	
    // Configure a ray according using 2 points
    // p1 is new origin
    void config(const PnD<N>& p1,
                const PnD<N>& p2,
                const RGBA& c = {0.0, 0.0, 0.0, 255.0});

    // Return a point on a ray
    PnD<N> operator()(double t) const;
};


template<std::size_t N>
std::ostream& operator<<(std::ostream& os, const Ray_<N>& r)
{
    os 	<< "origin: " << r.origin().transpose() << std::endl
    	<< "direction: " << r.direction().transpose() << std::endl
    	<< "color: r=" 	<< r.color().r << ", g=" << r.color().g 
    		 << ", b="  << r.color().b << ", a=" << r.color().a;
   
    return os;
}
template<std::size_t N>
void save(v::OutputArchive& archive, const Ray_<N>& r)
{
    archive
    ("origin", r.origin())
    ("direction", r.direction());
}
template<std::size_t N>
void load(v::InputArchive& archive, Ray_<N>& r)
{
    archive
    ("origin", r.origin())
    ("direction", r.direction());
}

using Ray = Ray_<3ul>;
using Ray3D = Ray_<3ul>;
using Ray2D = Ray_<2ul>;

//------------------------------------------------------------------------------
// Determine the t coefficient of a parametric line equation where its cross a plane
double compute_t_coef(const PlaneCoefficients& plane, const Ray3D& line);

// Determine the intersection point between a plane and a line
P3D line_plane_intersection(const PlaneCoefficients& plane, const Ray3D& ray);

// return the ditance between a point to a line
double line_point_distance(const Ray3D& ray, const P3D& point);
