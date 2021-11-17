#pragma once

#include "types.h"
#include "geometry/camera/plenoptic.h"
#include "geometry/observation.h"

#include "depth.h"
#include "strategy.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
void optimize_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const NeighborsIndexes& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
);

void optimize_depth_from_obs(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------	
	const BAPObservations& observations, /* (u,v,rho) */
	const PlenopticCamera& mfpc, const Image& scene
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void bruteforce_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const NeighborsIndexes& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
void gss_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const NeighborsIndexes& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
);
