#pragma once

#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/core/serialization/contiguous_containers.hpp> //support for std::vector

#include "geometry/observation.h"
#include "types.h"

V_DEFINE_PROPERTIES(ObservationsConfig)
(    
    V_DEFINE_PROPERTY(features, BAPObservations(0), "BAP Observations")
    V_DEFINE_PROPERTY(centers, 	MICObservations(0), "Center Observations")
	V_DEFINE_PROPERTY(corners, CBObservations(0), "Corner Observations")
)
