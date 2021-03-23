#pragma once

#include "types.h"

#include <libv/geometry/plane_equation.hpp>

#include "camera.h"
#include "pinhole.h"

#include "geometry/sensor.h"
#include "geometry/pose.h"
#include "geometry/ray.h"

#include "io/printer.h"

////////////////////////////////////////////ThinLensCamera//////////////////////////////////////
class ThinLensCamera final : public PinholeCamera
{
private:
    double aperture_; // aperture of the lens
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    ThinLensCamera(double f = 1.0, double aperture = 4.0, const Sensor& s = {}) : PinholeCamera{f, s}, aperture_{aperture} {}
    virtual ~ThinLensCamera() override {}
    
//Accessors	    
	double aperture() const { return aperture_; }
    double& aperture() { return aperture_; }   

//Computed parameters    
    double diameter() const { return std::fabs(this->focal()) / this->aperture(); }
    double radius() const { return diameter() / 2.; }
    
    // the plane equation coefficients
    // the plane coefficients
	Eigen::Matrix<double, 4, 1> plane() const
	{
		return v::plane_from_3_points(P3D{0., 0., 0.}, P3D{1., 0., 0.}, P3D{1., 1., 0.});
	};

	// the plane coefficients in WORLD coordinate system
	Eigen::Matrix<double, 4, 1> planeInWorld() const
	{
		return v::plane_from_3_points(from_coordinate_system_of(pose(), P3D{0., 0., 0.}),
		                          from_coordinate_system_of(pose(), P3D{1., 0., 0.}),
		                          from_coordinate_system_of(pose(), P3D{1., 1., 0.}));
	};
    
//Project and Raytrace
	bool project(const P3D& p3d_cam, P3D& projection) const
	{
		//the distance according to z axis
		const double dist2pt = std::fabs(p3d_cam[2]);

		if (dist2pt == 0.0)
		{
		    PRINT_WARN("ThinLensCamera::project: the point is on the lens plane!");
		    return false;
		}
		if (dist2pt == this->focal())
		{
		    PRINT_WARN("ThinLensCamera::project: the point is on the focal plane!");
		    return false;
		}

		// projecting the point
		projection = - this->focal() / (dist2pt - this->focal()) * p3d_cam;

		return true;
	}
 
    bool raytrace(const Ray3D& ray_in, Ray3D& ray_out) const 
    {
		auto is_on_disk = [](const P2D& p, double r) {
			return p.norm() <=  r; //std::sqrt(2.)
		};
		
		bool raytraced = false;
		
		// project the origin of the ray_in using the Thin Lens equation  
		P3D projected_point;
		if (this->project(ray_in.origin(), projected_point))
		{
		    // compute the intersection point between the ray and the lens
		    ray_out.origin() = line_plane_intersection(plane(), ray_in);

		    // Testing if the ray hit the lens
		    if (is_on_disk(ray_out.origin().head<2>(), this->radius()))
		    {
		        ray_out.config(ray_out.origin(), projected_point);
		        raytraced = true;
		    }
		/*
		    else
		    {
		        PRINT_DEBUG("ThinLensCamera::raytrace: the ray does not hit the lens!");
		    }
		*/
		}

		return raytraced;
    }   
};

inline std::ostream& operator<<(std::ostream& o, const ThinLensCamera& tcm)
{
    o << "Focal (mm) = " << tcm.focal() << std::endl
      << "Diameter (mm) = " << tcm.diameter() << std::endl
      << "Pose = {" << std::endl << tcm.pose() << "}" << std::endl;

    return o;
}
