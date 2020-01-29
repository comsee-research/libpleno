#include "corner.h"

#include "geometry/reprojection.h"

bool CornerReprojectionError::operator()(
	const Pose& camera_pose,
	const Pose& mla_pose,
	const MLA_t& mla,
	const Sensor& sensor,
	const ThinLensCamera& tcm, 
	const Distortions& distortions,
	ErrorType& error
) const
{    
    error.setZero();
	
	const P2D prediction = reproject_corner(
		pcm, camera_pose, checkerboard, observation
	);
	
	error[0] = observation.u - prediction[0];
	error[1] = observation.v - prediction[1];

    // mla is behind the sensor
    const double dist_sensor_mla = 1e2 * (mla_pose.translation().z() - sensor.pose().translation().z());
    if (dist_sensor_mla < 0.0)
    {
        error[0] += std::expm1(-dist_sensor_mla);
        error[1] += std::expm1(-dist_sensor_mla);
    }
    
    // focal is negatif
    const double f = tcm.focal();
    if (f < 0.0)
    {
        error[0] += std::expm1(-f);
        error[1] += std::expm1(-f);
    }

    return true;
}
