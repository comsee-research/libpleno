#include "bap.h"

#include "geometry/reprojection.h"

bool BlurAwarePlenopticReprojectionError::operator()(
	const Pose& camera_pose,
	const Pose& /* mla_pose */,
	const MLA_t& /*mla*/,
	const FocalLength& /*fml*/,
	const Sensor& /* sensor */,
	const ThinLensCamera& /* tcm */, 
	const Distortions& /* distortions */,
	ErrorType& error
) const
{    
    error.setZero();
	
	const P3D prediction = reproject_bapfeature(
		pcm, camera_pose, checkerboard, observation
	);
	
	error[0] = (observation.u 	- prediction[0]);
	error[1] = (observation.v 	- prediction[1]);
	error[2] = (observation.rho - prediction[2]);

    // mla is behind the sensor
    if (double d = 1e2 * pcm.d(observation.k, observation.l); d < 0.0)
    {
        error[0] += std::expm1(-d);
        error[1] += std::expm1(-d);
        error[2] += std::expm1(-d);
    }
	// focal is negatif
    if (double f = pcm.focal(); f < 0.0)
    { 
    	error[0] += std::expm1(-f);  
    	error[1] += std::expm1(-f);
    	error[2] += std::expm1(-f);
    }
    
    return true;
}
