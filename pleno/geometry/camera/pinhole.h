#pragma once

#include "types.h"

#include "camera.h"

#include "geometry/sensor.h"
#include "geometry/pose.h"
#include "geometry/ray.h"


////////////////////////////////////////////PinholeCamera//////////////////////////////////////
class PinholeCamera : public Camera
{
protected:
    double focal_; // The focal length (mm)
    	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    PinholeCamera(double f, const Sensor& s) : Camera{s}, focal_{f} {}
    virtual ~PinholeCamera() override {}
    
//Accessors	
	double focal() const { return focal_; }
    double& focal() { return focal_; }   
 
//Project and Raytrace
    bool project(const P3D& p3d_cam, P2D& pixel) const override 
    {
		Ray3D ray;
		ray.config(p3d_cam, pose().translation()); // CAMERA

		P3D p3d = line_plane_intersection(sensor().planeInWorld(), ray); // CAMERA
		p3d = to_coordinate_system_of(sensor().pose(), p3d); // SENSOR

		pixel = sensor().metric2pxl(p3d).head(2); //IMAGE XY
		xy2uv(pixel); //IMAGE UV

		return hit_the_sensor(pixel);
    }
    
    bool raytrace(const P2D& pixel, Ray3D& ray) const override 
    {
		bool is_projected = hit_the_sensor(pixel);
		
		P2D pix = pixel; //IMAGE UV
		uv2xy(pix); //IMAGE XY		
		
		P3D m {pix[0], pix[1], 0.0};
		m = sensor().pxl2metric(m); // SENSOR
		m = from_coordinate_system_of(sensor().pose(), m); // CAMERA

		ray.config(m, pose().translation());

		return is_projected;
    }
};

