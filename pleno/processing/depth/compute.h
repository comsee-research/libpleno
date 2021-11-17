#pragma once

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/depth/depthmap.h"
#include "geometry/observation.h"

#include "strategy.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_refined_depthmap(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_probabilistic_depthmap(
	DepthMap& dm,
	const PlenopticCamera& mfpc, const Image& scene, 
	std::size_t kinit, std::size_t linit,
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void compute_depthmap_from_obs(
	DepthMap& dm, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);
