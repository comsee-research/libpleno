#include "mic.h"

#include "geometry/reprojection.h"

bool MicroImageCenterReprojectionError::operator()(
	const Pose& /* mla_pose */, const MLA_t& /*g*/, const Sensor& /* sensor */, ErrorType& error
) const
{    
	constexpr double penalty = 1e4;
	error.setZero();
	
	const P2D center = reproject_miccenter(
		pcm, observation
	);
	
    error[0] = (observation[0] - center[0]); //IMAGE
    error[1] = (observation[1] - center[1]); //IMAGE

//CONSTRAINTS REGULARIZATION
    // mla is behind the sensor
    if (double d = 1e2 * pcm.d(observation.k, observation.l); d < 0.0)
    {
        error[0] += penalty; //std::expm1(-d);
        error[1] += penalty; // std::expm1(-d);
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
#endif 

    return true;
}

