#include "radius.h"

#include "geometry/reprojection.h"

template<typename CameraModel_t>
bool RadiusReprojectionError<CameraModel_t>::operator()(
	const Pose& camera_pose,
	const Pose& mla_pose,
	const MLA_t& mla,
	const FocalLength& f,
	const Sensor& sensor,
	const ThinLensCameraModel& tcm, 
	const Distortions& distortions,
	ErrorType& error
) const
{    
    error.setZero();
    
    const double prediction = reproject_radius(
    	pcm, camera_pose, checkerboard, observation    
    );

	error[0] = observation.rho - prediction;	
	//error[0] *= 10.;
	
    // mla is behind the sensor
    const double dist_sensor_mla = 1e2 * (mla_pose.translation().z() - sensor.pose().translation().z());
    if (dist_sensor_mla < 0.0)
    {
        error[0] += std::expm1(-dist_sensor_mla);
    }
    
    // focal is negatif
    const double mlf = tcm.f();
    if (mlf < 0.0)
    {
        error[0] += std::expm1(-mlf);
        error[1] += std::expm1(-mlf);
    }
	
	//error[0] = 0.;
	
    return true;
}

template class RadiusReprojectionError<MultiFocusPlenopticCamera>;
