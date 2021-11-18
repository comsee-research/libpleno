#pragma once

#include "types.h"

#include "geometry/camera/plenoptic.h"

#include "geometry/distortions.h"
#include "geometry/observation.h"
#include "geometry/internals.h"
#include "geometry/pose.h"

#include "object/checkerboard.h"
#include "object/constellation.h"

#include "geometry/depth/depthmap.h"

#include "processing/tools/functions.h"

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

//******************************************************************************
//******************************************************************************
//******************************************************************************
void calibration_LidarPlenopticCamera(                        
	CalibrationPose& pose, /* out */                   
	const PlenopticCamera& model, /* in */   
	const PointsConstellation& constellation,
	const BAPObservations& observations, /* (u,v,rho?) */
	const Image& scene
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
template <typename ScalingFunction>
void calibration_depthScaling(                        
	ScalingFunction& scaling,  
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, DepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
);

#include "scaling.hpp"

//******************************************************************************
//******************************************************************************
//******************************************************************************
