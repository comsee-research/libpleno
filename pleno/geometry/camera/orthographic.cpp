#include "orthographic.h"

#include "geometry/pose.h"
#include "io/printer.h"

//Ctor/Dtor
OrthographicCamera::OrthographicCamera(double f, const Sensor& s) : PinholeCamera{f, s} {}
OrthographicCamera::~OrthographicCamera() {}

//Project and Raytrace
bool OrthographicCamera::project(const P3D& /*p3d_cam*/, P2D& /*pixel*/) const 
{
	PRINT_ERR("OrthographicCamera::project not implemented yet.");
	return false;
}

bool OrthographicCamera::raytrace(const P2D& pixel, Ray3D& ray) const 
{
	bool is_projected = hit_the_sensor(pixel);
	
	P2D pix = pixel; //IMAGE UV
	uv2xy(pix); //IMAGE XY	

	P3D p_metric = sensor().pxl2metric(P3D{pix[0], pix[1], 0.0}); // SENSOR
	p_metric = from_coordinate_system_of(sensor().pose(), p_metric); // CAMERA

	P3D out_point = p_metric;
	out_point[2] += 1.0;

	ray.config(p_metric, out_point);

	return is_projected;
}
