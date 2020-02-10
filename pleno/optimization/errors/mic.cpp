#include "mic.h"

#include "geometry/reprojection.h"

bool MicroImageCenterReprojectionError::operator()(
	const Pose& mla_pose, const MLA_t& g, const Sensor& sensor, ErrorType& error
) const
{    
	error.setZero();
	
	const P2D center = reproject_miccenter(
		pcm, observation
	);
	
    error[0] = observation[0] - center[0]; //IMAGE
    error[1] = observation[1] - center[1]; //IMAGE

    // If mla is behind the sensor
    // increase error exponentially till it comes back in front of the sensor
    const double dist_sensor_mla = 1e2 * (mla_pose.translation().z() - sensor.pose().translation().z());
    if (dist_sensor_mla < 0.0)
    {
        error[0] += std::expm1(-dist_sensor_mla);
        error[1] += std::expm1(-dist_sensor_mla);
    }

    return true;
}

