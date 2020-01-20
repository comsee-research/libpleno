#include "bap.h"

#include "geometry/reprojection.h"

bool BlurAwarePlenopticReprojectionError::operator()(
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
	
	const P3D prediction = reproject_bapfeature(
		pcm, camera_pose, checkerboard, observation
	);
	
	error[0] = observation.u - prediction[0];
	error[1] = observation.v - prediction[1];
	
	if(not use_corner_only)	
		error[2] =  /* 1e1 * */ (std::fabs(observation.rho) - std::fabs(prediction[2]));

	if(use_regularization)
	{
		//Regularization
		const double d = std::fabs(pcm.mla().pose().translation()[2] - pcm.sensor().pose().translation()[2]);
		const double D = std::fabs(pcm.mla().pose().translation()[2]);
		const double F = pcm.main_lens().f();
		const double f0 = pcm.mla().f(0);
		const double f1 = pcm.mla().f(1);
		const double f2 = pcm.mla().f(2);
		const double m = pcm.params().m;
		const double c0 = pcm.params().c_prime[0];
		const double c1 = pcm.params().c_prime[1];
		const double c2 = pcm.params().c_prime[2];
		const double kappa = pcm.params().kappa;
		const double kappa_approx = kappa * (D / (D + d));
		
		double reg = std::fabs(pcm.mla().edge_length()[0] - kappa_approx)
			+ std::fabs(((d * F) / (2. * D)) - m)
			+ std::fabs( (1. / f0) * kappa_approx * (d / 2.) - c0) 
			+ std::fabs( (1. / f1) * kappa_approx * (d / 2.) - c1) 
			+ std::fabs( (1. / f2) * kappa_approx * (d / 2.) - c2) 
			+ 0.;
			
		//DEBUG_VAR(reg);
		error[3] = 0.01 * sensor.metric2pxl(reg);
	}
	
	//PRINT_DEBUG("Reprojected radius = " << prediction[2] << " -- Theorical radius = " << observation.rho);

    // mla is behind the sensor
    const double dist_sensor_mla = 1e2 * (mla_pose.translation().z() - sensor.pose().translation().z());
    if (dist_sensor_mla < 0.0)
    {
        error[0] += std::expm1(-dist_sensor_mla);
        error[1] += std::expm1(-dist_sensor_mla);
		if(not use_corner_only)
        	error[2] += std::expm1(-dist_sensor_mla);
    }

    return true;
}
