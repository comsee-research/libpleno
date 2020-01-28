#pragma once //from charlib

#include "types.h"

template<std::size_t N>
class Ray_
{
public:
	using Vector = Eigen::Matrix<double, N, 1>;

private: 
    Vector _origin;
    Vector _direction;
    RGBA _color;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //Constructor
    Ray_(const Vector& o = Vector::Zero(),
         const Vector& d = Vector::Zero(),
         const RGBA& c = {0.0, 0.0, 0.0, 255.0});

    ~Ray_();
    
	Vector & origin();
	const Vector & origin() const;
	
	Vector & direction();
	const Vector & direction() const;
	
	RGBA & color();
	const RGBA & color() const;
	
    // Configure a ray according using 2 points
    // p1 is new origin
    void config(const Vector& p1,
                const Vector& p2,
                const RGBA& c = {0.0, 0.0, 0.0, 255.0});

    // Return a point on a ray
    Vector operator()(const double t) const;
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
double compute_t_coef(const Eigen::Matrix<double, 4, 1>& plane, const Ray3D& line);

// Determine the intersection point between a plane and a line
P3D line_plane_intersection(const Eigen::Matrix<double, 4, 1>& plane, const Ray3D& ray);

// return the ditance between a point to a line
double line_point_distance(const Ray3D& ray, const P3D& point);
