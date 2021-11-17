#pragma once

#include "geometry/camera/plenoptic.h"

#include "types.h"

std::pair<double,double> 
initialize_min_max_distance(const PlenopticCamera& mfpc);
