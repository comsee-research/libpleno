#pragma once

#include "types.h"
#include "geometry/camera/plenoptic.h"

#include "strategy.h"

template <typename Functors>
void make_functors(
	Functors& functors, const NeighborsIndexes& neighs,
	std::size_t ck, std::size_t cl,
	const PlenopticCamera& mfpc, const Image& scene, 
	ObservationsPairingStrategy mode,
	double cu = -1., double cv = -1.
);

template <typename Functors>
void make_functors_from_obs(
	Functors& functors,
	const PlenopticCamera& mfpc, const Image& scene, 
	const BAPObservations& observations
);
