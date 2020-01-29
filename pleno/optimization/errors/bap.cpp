#include "bap.h"

#include "geometry/reprojection.h"

bool BlurAwarePlenopticReprojectionError::operator()(
	const Pose& camera_pose,
	const Pose& mla_pose,
	const MLA_t& mla,
	const FocalLength& f,
	const Sensor& sensor,
	const ThinLensCamera& tcm, 
	const Distortions& distortions,
	ErrorType& error
) const
{    
    error.setZero();
	
	const P3D prediction = reproject_bapfeature(
		pcm, camera_pose, checkerboard, observation
	);
	
	error[0] = observation.u - prediction[0];
	error[1] = observation.v - prediction[1];
	error[2] =  /* 1e1 * */ (observation.rho - prediction[2]);

	//PRINT_DEBUG("Reprojected radius = " << prediction[2] << " -- Theorical radius = " << observation.rho);

    // mla is behind the sensor
    const double dist_sensor_mla = 1e2 * (mla_pose.translation().z() - sensor.pose().translation().z());
    if (dist_sensor_mla < 0.0)
    {
        error[0] += std::expm1(-dist_sensor_mla);
        error[1] += std::expm1(-dist_sensor_mla);
        error[2] += std::expm1(-dist_sensor_mla);
    }

    return true;
}
