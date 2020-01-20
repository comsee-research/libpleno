#include "generate.h"

#include "detection/detection.h"

void generate_bapfeatures(
	BAPObservations& observations,
	const CameraModel_t& model,
	const Pose& pose,
	const CheckerBoard& grid
)
{
	CBObservations corners;
	generate_corner(corners, model, pose, grid);
	
	observations = compute_bapfeatures(corners, model.mia(), model.params());
}

