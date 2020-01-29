#pragma once

#include "types.h"

#include "camera.h"

#include "geometry/sensor.h"
#include "geometry/pose.h"
#include "geometry/ray.h"

#include "io/printer.h"



////////////////////////////////////////////UnifiedCamera//////////////////////////////////////
class UnifiedCamera final : public Camera //From libv
{
private:
	P2D focals_; /// The focal lengths.
	P2D center_; /// The principal point.
	double xi_;  /// The distortion parameter.
    	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    UnifiedCamera(double fx=1.0, double fy=1.0, double u0=0.0, double v0=0.0, double xi=0.0) : focals_{fx, fy}, center_{u0, v0}, xi_{xi} {}
    virtual ~UnifiedCamera() override {}
    
//Accessors	
	const P2D& focal() const { return focals_; }
	P2D& focal() { return focals_; }
	
	const P2D& center() const { return center_; }
	P2D& center() { return center_; }
	
//Project and Raytrace
    bool project(const P3D& p3d_cam, P2D& pixel) const override 
    {
		// project the point on the sphere
		P3D p = p3d_cam.normalized();

		// bail out if the point is behind the camera
		if((xi_ > 1 && xi_ * p.z() <= -1) || (xi_ >= 0 && xi_ <= 1 && p.z() <= -xi_)) return false;

		p.z() += xi_;

		pixel = p.head<2>().array() * focals_.array() / p.z() + center_.array();
		return true;
    }
    
    bool raytrace(const P2D& pixel, Ray3D& ray) const override 
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
};
