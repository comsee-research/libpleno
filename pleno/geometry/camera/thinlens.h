#pragma once

#include "geometry/sensor.h"
#include "geometry/camera/models.h"

class ThinLensCamera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ThinLensCameraModel tcm;
    Sensor sensor;

    ThinLensCamera(const ThinLensCameraModel& t, const Sensor& s)
    : tcm(t), sensor(s)
    {}

    bool project(const P3D& p3d_cam, P2D& pixel) const
    {
		P3D proj;
		bool is_projected = tcm.project(p3d_cam, proj); // THINLENS
		proj = from_coordinate_system_of(tcm.pose(), proj); // CAMERA

		// linking proj and mainlens center
		Ray3D ray;
		ray.config(tcm.pose().translation(), proj); // CAMERA

		proj = line_plane_intersection(sensor.planeInWorld(), ray); // CAMERA
		proj = to_coordinate_system_of(sensor.pose(), proj); // SENSOR

		pixel = sensor.metric2pxl(proj).head(2); // IMAGE XY
		
//FIXME:TEST_INVERSION      	
    	xy2uv(pixel); //IMAGE UV

		return is_projected;
	}
	
    bool raytrace(const P2D& pixel, Eigen::Vector3d& direction) const
	{
//FIXME:TEST_INVERSION    
		P2D p = pixel; // IMAGE UV
		uv2xy(p);// IMAGE XY
		
		P3D m {p[0], p[1], 0.0}; 
		m = sensor.pxl2metric(m); // SENSOR
		m = from_coordinate_system_of(sensor.pose(), m); // CAMERA

		Ray3D r;
		r.config(m, tcm.pose().translation()); // CAMERA
		direction = r.direction();

		return true;
	}

	void uv2xy(P2D& puv) const { puv[0] = sensor.width()-1 - puv[0]; puv[1] = sensor.height()-1 - puv[1];}
	void xy2uv(P2D& pxy) const { pxy[0] = sensor.width()-1 - pxy[0]; pxy[1] = sensor.height()-1 - pxy[1];}
};

inline std::ostream& operator<<(std::ostream& o, const ThinLensCamera& tc)
{
    o 	<< " {" << std::endl
    	<< "\tTCM = {" << std::endl << tc.tcm << std::endl << "}," << std::endl
    	<< "\tSensor = {" << std::endl << tc.sensor << std::endl << "}" << std::endl
    	<< "}";

    return o;
}
