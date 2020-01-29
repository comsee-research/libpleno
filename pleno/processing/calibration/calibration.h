#pragma once

#include "types.h"

#include "geometry/camera/models.h"
#include "geometry/observation.h"

#include "object/checkerboard.h"

void calibration_MIA(
	MicroImagesArray& grid, /* out */
	const MICObservations& centers /* c_{k,l} */
);

template<bool useCornerOnly = false>
void calibration_MFPC(                   
	CalibrationPoses& poses, /* out */                                 
	PlenopticCamera& model, /* out */
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const MICObservations& centers, /* c_{k,l} */
	const std::vector<Image>& pictures /* for GUI only */
);

void calibration_ExtrinsicsMFPC(                        
	CalibrationPoses& poses, /* out */                   
	const PlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const std::vector<Image>& pictures, /* for GUI only */
	bool useCornerOnly
);

