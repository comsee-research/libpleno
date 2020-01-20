#pragma once

#include "types.h"

#include "geometry/camera/mfpc.h"
#include "geometry/observation.h"

#include "geometry/object/checkerboard.h"

using CameraModel_t = MultiFocusPlenopticCamera;

void generate_poses(
	CalibrationPoses& poses, /* out */
	const CameraModel_t& model, 
	const CheckerBoard& grid, 
	std::size_t n
);

void generate_pose(Pose& pose, double min = 100., double max = 2000.);

void generate_mic(
	MICObservations& observations, 
	const CameraModel_t& model
);

void generate_corner(
	CBObservations& observations, 
	const CameraModel_t& model,
	const Pose& pose,
	const CheckerBoard& grid
);

void generate_bapfeatures(
	BAPObservations& observations,
	const CameraModel_t& model,
	const Pose& pose,
	const CheckerBoard& grid
);


