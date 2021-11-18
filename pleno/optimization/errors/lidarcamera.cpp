#include "lidarcamera.h"

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool LidarCameraCornerReprojectionError::operator()(
	const Pose& pose,
	ErrorType& error
) const
{    
    error.setZero();
    
    const P3D p3d = constellation.get(observation.cluster); // LIDAR 
    const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

    P2D prediction;
    [[maybe_unused]] const bool is_projected = pcm.project(p3d_cam, observation.k, observation.l, prediction);
	
	error[0] = observation.u - prediction[0];
	error[1] = observation.v - prediction[1];

    return true;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
bool LidarCameraBlurAwarePlenopticReprojectionError::operator()(
	const Pose& pose,
	ErrorType& error
) const
{    
    error.setZero();
    
    const P3D p3d = constellation.get(observation.cluster); // LIDAR
    const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

    P3D prediction;
    [[maybe_unused]] const bool is_projected = pcm.project(p3d_cam, observation.k, observation.l, prediction);
	
	error[0] = observation.u - prediction[0];
	error[1] = observation.v - prediction[1];
	error[2] = observation.rho - prediction[2];

    return true;
}
