#include "extrinsics_bap.h"

bool ExtrinsicsBlurAwarePlenopticReprojectionError::operator()(
	const Pose& camera_pose,
	ErrorType& error
) const
{    
    error.setZero();
    
    const P3D p3d = checkerboard.nodeInWorld(observation.cluster); // WORLD
    const P3D p3d_cam = to_coordinate_system_of(camera_pose, p3d); // CAMERA

    P3D prediction;
    const bool is_projected = pcm.project(p3d_cam, observation.k, observation.l, prediction);
	
	error[0] = observation.u - prediction[0];
	error[1] = observation.v - prediction[1];
	if(not use_corner_only)
		error[2] = observation.rho - prediction[2];

    return true;
}
