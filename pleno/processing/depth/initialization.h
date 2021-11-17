#pragma once

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/mia.h"

#include "depth.h"
#include "strategy.h"

std::pair<double,double> 
initialize_min_max_distance(const PlenopticCamera& mfpc);

IndexPair initialize_kl(
	std::size_t i, std::size_t n, 
	const MIA& mia, 
	InitStrategy mode = InitStrategy::REGULAR_GRID
);

void initialize_depth(
	DepthHypothesis& hypothesis,
	//--------------------------------------------------------------------------
	const NeighborsIndexes& neighs, 
	const PlenopticCamera& mfpc, const Image& scene, 
	const DepthEstimationStrategy& strategies
);
