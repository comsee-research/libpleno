#pragma once

#include "types.h"
#include "io/archive.h"

using PlaneCoefficients = Eigen::Matrix<double, 4, 1>; //Coefficients (a,b,c,d) such as ax + by + cz = d 

/**
	Compute the signed distance between the plane and the point with a²+b²+c²=1
**/
double compute_signed_distance(const PlaneCoefficients& normalized_plane_eqn, const P3D& p3d);

/**
	Compute the distance between the plane and the point with a²+b²+c²=1
**/
double compute_distance(const PlaneCoefficients& normalized_plane_eqn, const P3D& p3d);

/**
	Compute a plane equation from 3 3D points: ax + by + cz = d
**/
PlaneCoefficients plane_from_3_points(const P3D& p1, const P3D& p2, const P3D& p3);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class Plane 
{
	PlaneCoefficients coeff_;
	P3D origin_;

public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW 

	Plane();
	Plane(const PlaneCoefficients& coeffs);
	Plane(const PlaneCoefficients& coeffs, const P3D& o);
	Plane(const P3D& p1, const P3D& p2, const P3D& p3);
	
	Plane(const Plane& o);
	Plane(Plane&& o);
	
	virtual ~Plane();
	
	const PlaneCoefficients& coeff() const;
	PlaneCoefficients& coeff();
	
	const P3D& origin() const;
	P3D& origin();
	
	double a() const;
	double b() const;
	double c() const;
	double d() const;
	
	P3D e1() const;
	P3D e2() const;
	P3D e3() const;
	
	P3D e(std::size_t i) const;
	
	P3D normal() const;
	
	double dist(const P3D& p = P3D::Zero()) const;
	P3D position() const;
	
protected:
	////////////////////////////////////////////////////////////////////////////
	friend void save(v::OutputArchive& archive, const Plane& plane);
	friend void load(v::InputArchive& archive, Plane& plane);	
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void save(v::OutputArchive& archive, const Plane& plane);
void load(v::InputArchive& archive, Plane& plane);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator<<(std::ostream& os, const Plane& plane);
