#pragma once

#include "types.h"

#include "camera.h"
#include "pinhole.h"

#include "geometry/sensor.h"
#include "geometry/ray.h"
#include "geometry/plane.h"

////////////////////////////////////////////ThinLensCamera//////////////////////////////////////
class ThinLensCamera final : public PinholeCamera
{
private:
    double aperture_; // aperture of the lens
    
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    ThinLensCamera(double f = 1.0, double aperture = 4.0, const Sensor& s = {});
    ~ThinLensCamera() override;
    
//Accessors	    
	double aperture() const;
    double& aperture(); 

//Computed parameters    
    double diameter() const;
    double radius() const;
    
    // the plane coefficients
	PlaneCoefficients plane() const;
	// the plane coefficients in WORLD coordinate system
	PlaneCoefficients planeInWorld() const;
    
//Project and Raytrace
	bool project(const P3D& p3d_cam, P3D& projection) const;
    bool raytrace(const Ray3D& ray_in, Ray3D& ray_out) const;
};

std::ostream& operator<<(std::ostream& o, const ThinLensCamera& tcm);
