#pragma once

#include "io/archive.h"

#include "geometry/pose.h"

V_DEFINE_PROPERTIES(ThinLensConfig)
(
    V_DEFINE_PROPERTY(f, double(-1.), "Thin Lens Focal length")
    V_DEFINE_PROPERTY(aperture, double(-1.), "Thin Lens Aperture f/2.8, f/4, ...")
    V_DEFINE_PROPERTY(diameter, double(-1.), "Thin Lens Diameter (A = F/N)")
    V_DEFINE_PROPERTY(pose, Pose(), "Thin Lens Pose")
)
