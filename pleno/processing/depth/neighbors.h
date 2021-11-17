#pragma once

#include <vector>
#include <map>

#include "geometry/mia.h"
#include "geometry/sensor.h"

#include "types.h"

NeighborsIndexes
inner_ring(const MIA& mia, std::size_t k, std::size_t l);

NeighborsIndexes neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);

std::map<double, NeighborsIndexes> neighbors_by_rings(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);

NeighborsIndexes pixels_neighbors(
	const MIA& mia, std::size_t W, std::size_t H, std::size_t k, std::size_t l
);

#if 1
NeighborsIndexes half_neighbors(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);

std::map<double, NeighborsIndexes> half_neighbors_by_rings(
	const MIA& mia, std::size_t k, std::size_t l, 
	double v, double minv = 5., double maxv = 12.
);
#endif
