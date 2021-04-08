#pragma once

#include "io/archive.h"

#include "geometry/pose.h"

V_DEFINE_PROPERTIES(SensorConfig)
(
    V_DEFINE_PROPERTY(pose, Pose(), "Sensor Pose")
    V_DEFINE_PROPERTY(height, (std::size_t(0u)), "Number of rows in the array)")
    V_DEFINE_PROPERTY(width, (std::size_t(0u)), "Number of cols in the array")
    V_DEFINE_PROPERTY(scale, (double(-1.0)), "Size of a pixel (mm)")
)

