#pragma once

#include "types.h"

#include "geometry/internals.h"
#include "geometry/mia.h"
#include "geometry/observation.h"

MICObservations 
detection_mic(const Image& raw);

CBObservations
detection_corners(const Image& raw, const MIA& mia, const InternalParameters& params);

BAPObservations
detection_bapfeatures(const Image& raw, const MIA& mia, const InternalParameters& params);

#include "detection/bapfeatures.hpp"
