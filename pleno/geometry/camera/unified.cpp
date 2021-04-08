#include "unified.h"


//Ctor/Dtor
UnifiedCamera::UnifiedCamera(double fx, double fy, double u0, double v0, double xi) : focals_{fx, fy}, center_{u0, v0}, xi_{xi} {}
UnifiedCamera::~UnifiedCamera() {}
    
//Accessors	
const P2D& UnifiedCamera::focal() const { return focals_; }
P2D& UnifiedCamera::focal() { return focals_; }
	
const P2D& UnifiedCamera::center() const { return center_; }
P2D& UnifiedCamera::center() { return center_; }
	
//Project and Raytrace
bool UnifiedCamera::project(const P3D& p3d_cam, P2D& pixel) const 
{
	// project the point on the sphere
	P3D p = p3d_cam.normalized();

	// bail out if the point is behind the camera
	if((xi_ > 1 && xi_ * p.z() <= -1) || (xi_ >= 0 && xi_ <= 1 && p.z() <= -xi_)) return false;

	p.z() += xi_;

	pixel = p.head<2>().array() * focals_.array() / p.z() + center_.array();
	return true;
}
    
bool UnifiedCamera::raytrace(const P2D& pixel, Ray3D& ray) const 
{
	P2D p = (pixel - center_).array() / focals_.array();
	
	const double a = p.squaredNorm();
	const double b = 1.0 + a * (1.0 - xi_ * xi_);
	
	if (b < 0.0) 
	{
		PRINT_WARN("UnifiedCamera::project: the point is outside the image");
		return false;
	}
	
	double eta = (xi_ + std::sqrt(b)) / (1.0 + a);
	P3D p_in_cam{eta * p.x(), eta * p.y(), eta - xi_};
	
	ray.config(P3D::Zero(), p_in_cam);

	return true;
}
