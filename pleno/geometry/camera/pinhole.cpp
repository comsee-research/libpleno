#include "pinhole.h"

#include "geometry/pose.h"

//Ctor/Dtor
PinholeCamera::PinholeCamera(double f, const Sensor& s) : Camera{s}, focal_{f} {}
PinholeCamera::~PinholeCamera() {}
    
//Accessors	
double PinholeCamera::focal() const { return focal_; }
double& PinholeCamera::focal() { return focal_; }   

//Project and Raytrace
bool PinholeCamera::project(const P3D& p3d_cam, P2D& pixel) const 
{
	Ray3D ray;
	ray.config(p3d_cam, pose().translation()); // CAMERA

	P3D p3d = line_plane_intersection(sensor().planeInWorld(), ray); // CAMERA
	p3d = to_coordinate_system_of(sensor().pose(), p3d); // SENSOR

	pixel = sensor().metric2pxl(p3d).head<2>(); //IMAGE XY
	xy2uv(pixel); //IMAGE UV

	return hit_the_sensor(pixel);
}

bool PinholeCamera::raytrace(const P2D& pixel, Ray3D& ray) const 
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
