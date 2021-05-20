#pragma once

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/observation.h"

#include "geometry/object/checkerboard.h"
#include "geometry/object/plate.h"

using CameraModel_t = PlenopticCamera;

void generate_poses(
	CalibrationPoses& poses, /* out */
	const CameraModel_t& model, 
	const CheckerBoard& grid, 
	std::size_t n, double min = 100., double max = 2000.
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

void generate_image(
	Image& raw,
	const CameraModel_t& model,
	const Plate& scene,
	bool blur = true
);

void generate_image(
	Image& raw,
	const CameraModel_t& model,
	const Plate& scene,
	std::size_t nrays,
	bool vignetting = true
);

