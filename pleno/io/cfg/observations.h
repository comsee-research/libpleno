#pragma once

#include "io/archive.h"

#include "geometry/observation.h"
#include "types.h"

V_DEFINE_PROPERTIES(ObservationsConfig)
(    
    V_DEFINE_PROPERTY(features, BAPObservations(0), "BAP Observations")
    V_DEFINE_PROPERTY(centers, 	MICObservations(0), "Center Observations")
	V_DEFINE_PROPERTY(corners, CBObservations(0), "Corner Observations")
)
