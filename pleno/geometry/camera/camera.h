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
	Sensor 	sensor_;
	Pose	pose_;
	
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
//Ctor/Dtor
	Camera(const Sensor& s = {}, const Pose& p = {});
	virtual ~Camera();

//Accessors	
	const Sensor& sensor() const;
    Sensor& sensor();
    
    Pose& pose();
	const Pose& pose() const;

//Project and Raytrace
	virtual bool project(const P3D& p3d_cam, P2D& pixel) const = 0;
  	virtual bool raytrace(const P2D& pixel, Ray3D& ray) const = 0;

//Space convertion	(Image Space / Sensor Space)
	void uv2xy(P2D& puv) const;
	void xy2uv(P2D& pxy) const;
	
protected:
//Helper functions
	bool hit_the_sensor(const P2D& p) const;
};
