#include "plane.h"

#include <Eigen/Geometry>
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/**
	Compute the signed distance between the plane and the point with a²+b²+c²=1
**/
double compute_signed_distance(const PlaneCoefficients& normalized_plane_eqn, const P3D& p3d)
{
  Eigen::Matrix<double, 1, 4> tp1;
  tp1[0] = p3d[0];
  tp1[1] = p3d[1];
  tp1[2] = p3d[2];
  tp1[3] = 1;
  return (tp1 * normalized_plane_eqn);
}

/**
	Compute the distance between the plane and the point with a²+b²+c²=1
**/
double compute_distance(const PlaneCoefficients& normalized_plane_eqn, const P3D& p3d)
{
  return std::fabs(compute_signed_distance(normalized_plane_eqn,p3d));
}

/**
	Compute a plane equation from 3 3D points: ax + by + cz = d
**/
PlaneCoefficients plane_from_3_points(const P3D& p1, const P3D& p2, const P3D& p3)
{
    PlaneCoefficients eq;
    eq << ((p2-p1).cross(p3-p1)).normalized(), 0;
    eq(3) = - compute_signed_distance(eq,p1);
    return eq;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Plane::Plane() 
{ 
	coeff_ << 0., 0., 1., 0.; //unit normal vector
	origin_ << 0., 0., 0.; //origin
}

Plane::Plane(const PlaneCoefficients& coeffs) 
	: coeff_(coeffs), origin_(coeffs.head<3>() * coeffs(3) / coeffs.head<3>().squaredNorm()) 
{
}

Plane::Plane(const PlaneCoefficients& coeffs, const P3D& o) 
	: coeff_(coeffs), origin_(o) 
{
}

Plane::Plane(const P3D& p1, const P3D& p2, const P3D& p3) 
	: coeff_(plane_from_3_points(p1,p2,p3)), origin_((p1 + p2 + p2) / 3.) 
{
}
	
Plane::Plane(const Plane& o) : coeff_(o.coeff_), origin_(o.origin_) {}
Plane::Plane(Plane&& o) : coeff_(std::move(o.coeff_)), origin_(std::move(o.origin_)) {}

Plane::~Plane() {}
	
const PlaneCoefficients& Plane::coeff() const { return coeff_; }
PlaneCoefficients& Plane::coeff() { return coeff_; }

const P3D& Plane::origin() const { return origin_; }
P3D& Plane::origin() { return origin_; }

double Plane::a() const { return coeff_(0); }
double Plane::b() const { return coeff_(1); }
double Plane::c() const { return coeff_(2); }
double Plane::d() const { return coeff_(3); }

P3D Plane::e1() const { return P3D{c() - b(), a() - c(), b() - a()}.normalized(); }
P3D Plane::e2() const { 
	return P3D{	a() * (b() + c()) - b() * b() - c() * c(), 
				b() * (a() + c()) - a() * a() - c() * c(),
				c() * (a() + b()) - a() * a() - b() * b()
			}.normalized(); 
}
P3D Plane::e3() const { return coeff().head<3>().normalized(); }

P3D Plane::e(std::size_t i) const 
{ 
	assert(i > 0 and i <= 3); 
	if (i==1) 		return e1(); 
	else if (i==2) 	return e2(); 
	else 			return e3(); 
}

P3D Plane::normal() const { return e3(); }

double Plane::dist(const P3D& p) const 
{ 
	return (a() * p.x() + b() * p.y() + c() * p.z() - d()) / coeff().head<3>().norm();
}	
P3D Plane::position() const 
{ 
	return P3D{coeff().head<3>() * d() / coeff().head<3>().squaredNorm()}; 
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void save(v::OutputArchive& archive, const Plane& plane)
{
	archive
		("coeff", plane.coeff())
		("origin", plane.origin());
}

void load(v::InputArchive& archive, Plane& plane)
{
	archive
		("coeff", plane.coeff())
		("origin", plane.origin());
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, const Plane& plane)
{
	os << plane.a() << " * x + " << plane.b() << " * y + " << plane.c() << " * z = " << plane.d();
	return os; 
}
