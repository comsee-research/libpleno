#include "radius.h"

#include "geometry/reprojection.h"

#include "io/printer.h"

bool BlurRadiusReprojectionError::operator()(
	const Pose& camera_pose,
	const Pose& /* mla_pose */,
	const MLA_t& /* mla */,
	const FocalLength& /* fml */,
	const Sensor& /* sensor */,
	const ThinLensCamera& /* tcm */, 
	const Distortions& /* distortions */,
	ErrorType& error
) const
{    
	DEBUG_ASSERT((pcm.mla().I()>0u), "Can't reproject radius in BAP feature.");
	
	constexpr double scalerr = 1.; 
	constexpr double penalty = 1e4;
    error.setZero();
    
    const double prediction = reproject_radius(
    	pcm, camera_pose, checkerboard, observation    
    );

	error[0] = (observation.rho - prediction) * scalerr;

//CONSTRAINTS REGULARIZATION	
    // mla is behind the sensor
    if (double d = 1e2 * pcm.d(observation.k, observation.l); d < 0.0)
    { 
    	error[0] += penalty; //std::expm1(-d);
    }
    
    // focals are negatif
    if (double f = pcm.focal(); f < 0.0) 
    { 
    	error[0] += penalty; //std::expm1(-f); 
    }
    
#if 0
    if (fml.f < 0.0) { error[0] += 1e4; }
    
    // prevent switching mode
    if (pcm.mode() == PlenopticCamera::Mode::Keplerian and (std::fabs(fml.f) > pcm.d(observation.k, observation.l))) { error[0] += penalty; }
    if (pcm.mode() == PlenopticCamera::Mode::Galilean and (std::fabs(fml.f) < pcm.d(observation.k, observation.l))) { error[0] += penalty; }
    
    // sensor have move too far
    constexpr double ratio = 0.333;
    if (P2D pp = pcm.pp(); pp.x() < pcm.sensor().width() * ratio or pp.x() > pcm.sensor().width() * (1. - ratio) 
    	or pp.y() < pcm.sensor().height() * ratio or pp.y() > pcm.sensor().height() * (1. - ratio))
    {
   	 	error[0] += penalty; //std::expm1(-pp.x());
    }
	
    //pose is too far
	if (double z = std::fabs(camera_pose.translation().z()); z > pcm.distance_focus() - pcm.focal())
	{
   	 	error[0] += penalty; // std::expm1(-z);
	}	
	
	//focal planes are too far
	if (double fp = pcm.focal_plane(pcm.mla().type(observation.k, observation.l), observation.k, observation.l); fp > pcm.distance_focus() - pcm.focal())
	{
   	 	error[0] += penalty; // std::expm1(-z);
	}
#endif	    
    return true;
}
