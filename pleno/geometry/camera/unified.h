#pragma once

#include "types.h"

#include "camera.h"

#include "geometry/sensor.h"
#include "geometry/pose.h"
#include "geometry/ray.h"

#include "io/printer.h"



////////////////////////////////////////////UnifiedCamera//////////////////////////////////////
class UnifiedCamera final : public Camera //From libv
{
private:
	P2D focals_; /// The focal lengths.
	P2D center_; /// The principal point.
	double xi_;  /// The distortion parameter.
    	
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//Ctor/Dtor
    UnifiedCamera(double fx=1.0, double fy=1.0, double u0=0.0, double v0=0.0, double xi=0.0);
    ~UnifiedCamera() override;
    
//Accessors	
	const P2D& focal() const;
	P2D& focal();
	
	const P2D& center() const;
	P2D& center();
	
//Project and Raytrace
    bool project(const P3D& p3d_cam, P2D& pixel) const override;
    bool raytrace(const P2D& pixel, Ray3D& ray) const override;
};
