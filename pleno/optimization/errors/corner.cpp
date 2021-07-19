#include "corner.h"

#include "geometry/reprojection.h"

bool CornerReprojectionError::operator()(
	const Pose& camera_pose,
	const Pose& /* mla_pose */,
	const MLA_t& /* mla */,
	const Sensor& /* sensor */,
	const ThinLensCamera& /* tcm */, 
	const Distortions& /* distortions */,
	ErrorType& error
) const
{    
    constexpr double penalty = 1e4;
    error.setZero();
	
	const P2D prediction = reproject_corner(
		pcm, camera_pose, checkerboard, observation
	);
	
	error[0] = (observation.u - prediction[0]);
	error[1] = (observation.v - prediction[1]);

//CONSTRAINTS REGULARIZATION
    // mla is behind the sensor
    if (double d = 1e2 * pcm.d(observation.k, observation.l); d < 0.0)
    {
        error[0] += penalty; //std::expm1(-d);
        error[1] += penalty; // std::expm1(-d);
    }
    
    // focal is negatif 
    if (double f = pcm.focal(); f < 0.0)
    {
        error[0] += penalty; // std::expm1(-f);
        error[1] += penalty; // std::expm1(-f);
    }

#if 0   
    // sensor have move too far
    constexpr double ratio = 0.333;
    if (P2D pp = pcm.pp(); pp.x() < pcm.sensor().width() * ratio or pp.x() > pcm.sensor().width() * (1. - ratio) 
    	or pp.y() < pcm.sensor().height() * ratio or pp.y() > pcm.sensor().height() * (1. - ratio))
    {
   	 	error[0] += penalty; // std::expm1(-pp.x());
        error[1] += penalty; // std::expm1(-pp.y());
    }
    
    //pose is too far
	if (double z = std::fabs(camera_pose.translation().z()); z > pcm.distance_focus() - pcm.focal())
	{
   	 	error[0] += penalty; // std::expm1(-z);
        error[1] += penalty; // std::expm1(-z);
	}	
#endif
	    
    return true;
}
