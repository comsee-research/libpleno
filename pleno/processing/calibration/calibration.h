#pragma once

#include "types.h"

#include "geometry/camera/plenoptic.h"

#include "geometry/distortions.h"
#include "geometry/observation.h"
#include "geometry/internals.h"

#include "object/checkerboard.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_MIA(
	MicroImagesArray& grid, /* out */
	const MICObservations& centers /* c_{k,l} */
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_PlenopticCamera(                   
	CalibrationPoses& poses, /* out */                                 
	PlenopticCamera& model, /* out */
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const MICObservations& centers, /* c_{k,l} */
	const IndexedImages& pictures /* for GUI only */
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_ExtrinsicsPlenopticCamera(                        
	CalibrationPoses& poses, /* out */                   
	const PlenopticCamera& model, /* in */   
	const CheckerBoard & grid,
	const BAPObservations& observations, /*  (u,v,rho) */
	const IndexedImages& pictures /* for GUI only */
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_relativeBlur(
	InternalParameters& internals,     /* out */ 
	const BAPObservations& observations, /*  (u,v,rho) */
	const IndexedImages& images
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_inverseDistortions(
	Distortions& invdistortions,
	const PlenopticCamera& mfpc,
	const CheckerBoards& scene
);
