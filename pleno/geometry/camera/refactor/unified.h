#pragma once

#include "types.h"

#include "camera.h"

#include "geometry/sensor.h"
#include "geometry/pose.h"
#include "geometry/ray.h"


////////////////////////////////////////////UnifiedCamera//////////////////////////////////////
class UnifiedCamera final : public Camera //From libv
{
private:
	P2D _focals; /// The focal lengths.
	P2D _center; /// The principal point.
	double _xi;  /// The distortion parameter.
    	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    UnifiedCamera(double fx,, double fy, double u0, double v0, double xi) : _focals{fx, fy}, _center{u0, v0}, _xi{xi} {}
    virtual ~UnifiedCamera() override {}
    
//Accessors	

//Project and Raytrace
    bool project(const P3D& p3d_cam, P2D& pixel) const override 
    {
		// project the point on the sphere
		P3D p = p3d_cam.normalized();

		// bail out if the point is behind the camera
		if((_xi > 1 && _xi * p.z() <= -1) || (_xi >= 0 && _xi <= 1 && p.z() <= -_xi)) return false;

		p.z() += _xi;

		pixel = p.head<2>().array() * _focals.array() / p.z() + _center.array();
		return true;
    }
    
    bool raytrace(const P2D& pixel, Ray3D& ray) const override 
    {
		P2D p = (pixel - _center).array() / _focals.array();
		
		const double a = p.squaredNorm();
		const double b = 1.0 + a * (1.0 - _xi * _xi);
		
		if (b < 0.0) 
		{
			PRINT_WARN("UnifiedCamera::project: the point is outside the image");
			return false;
		}
		
		double eta = (_xi + std::sqrt(b)) / (1.0 + a);
		P3D p_in_cam{eta * p.x(), eta * p.y(), eta - _xi};
		
		ray.config(P3D::Zero(), p_in_cam);

		return true;
    }
};
