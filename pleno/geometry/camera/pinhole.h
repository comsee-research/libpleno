#pragma once

#include "types.h"

#include "camera.h"

#include "geometry/sensor.h"
#include "geometry/ray.h"

////////////////////////////////////////////PinholeCamera//////////////////////////////////////
class PinholeCamera : public Camera
{
protected:
    double focal_; // The focal length (mm)
    	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    PinholeCamera(double f, const Sensor& s);
    ~PinholeCamera() override;
    
//Accessors	
	double focal() const;
    double& focal(); 
 
//Project and Raytrace
    bool project(const P3D& p3d_cam, P2D& pixel) const override;     
    bool raytrace(const P2D& pixel, Ray3D& ray) const override;
};

