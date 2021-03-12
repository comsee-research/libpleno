#include "radius.h"

#include "geometry/reprojection.h"

#include "io/printer.h"

bool BlurRadiusReprojectionError::operator()(
	const Pose& camera_pose,
	const Pose& mla_pose,
	const MLA_t& /* mla */,
	const FocalLength& /* f */,
	const Sensor& /* sensor */,
	const ThinLensCamera& tcm, 
	const Distortions& /* distortions */,
	ErrorType& error
) const
{    
	DEBUG_ASSERT((pcm.mla().I()>0u), "Can't reproject radius in BAP feature.");
	
    error.setZero();
    
    const double prediction = reproject_radius(
    	pcm, camera_pose, checkerboard, observation    
    );

	error[0] = observation.rho - prediction;	
	//error[0] *= 10.;
	
    // mla is behind the sensor
    const double dist_sensor_mla = 1e2 * (mla_pose.translation().z() - pcm.sensor().pose().translation().z());
    if (dist_sensor_mla < 0.0)
    {
        error[0] += std::expm1(-dist_sensor_mla);
    }
    
    // focal is negatif
    if (tcm.focal() < 0.0) { error[0] += std::expm1(-tcm.focal());  }
		
    return true;
}
