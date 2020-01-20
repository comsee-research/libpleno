#pragma once

#include "types.h"
#include "geometry/observation.h"

CBObservations clusterize(const CBObservations& obs, double eps, std::size_t min_pts, bool filter = false);
