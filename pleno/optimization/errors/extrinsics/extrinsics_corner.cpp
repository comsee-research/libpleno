#include "extrinsics_corner.h"

bool ExtrinsicsCornerReprojectionError::operator()(
	const Pose& camera_pose,
	ErrorType& error
) const
{    
    error.setZero();
    
    const P3D p3d = checkerboard.nodeInWorld(observation.cluster); // WORLD
    const P3D p3d_cam = to_coordinate_system_of(camera_pose, p3d); // CAMERA

    P2D prediction;
    const bool is_projected = pcm.project(p3d_cam, observation.k, observation.l, prediction);
	
	error[0] = observation.u - prediction[0];
	error[1] = observation.v - prediction[1];

    return true;
}
