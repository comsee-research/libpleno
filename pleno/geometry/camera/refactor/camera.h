#pragma once

#include "types.h"

#include "geometry/sensor.h"
#include "geometry/pose.h"
#include "geometry/ray.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
class Camera 
{	
protected:
	Sensor 	_sensor;
	Pose	_pose;
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
//Ctor/Dtor
	Camera(const Sensor& s = {}, const Pose& p = {}) : _sensor{s}, _pose{p} {}	
	virtual ~Camera() {}

//Accessors	
	const Sensor& sensor() const { return _sensor; }
    Sensor& sensor() { return _sensor; }
    
    Pose& pose() { return _pose; }
	const Pose& pose() const { return _pose; }

//Project and Raytrace
	virtual bool project(const P3D& p3d_cam, P2D& pixel) const = 0;
  	virtual bool raytrace(const P2D& pixel, Ray3D& ray) const = 0;

//Space convertion	(Image Space / Sensor Space)
	void uv2xy(P2D& puv) const { puv[0] = sensor.width()-1 - puv[0]; puv[1] = sensor.height()-1 - puv[1];}
	void xy2uv(P2D& pxy) const { uv2xy(pxy); }
	
protected:
//Helper functions
	bool hit_the_sensor(const P2D& p) const {
		return p[0] >= 0.0 and p[1] >= 0.0 and p[0] < _sensor.width() and p[1] < _sensor.height();
	};
};
