#pragma once

#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/core/serialization/contiguous_containers.hpp> //support for std::vector

#include "types.h"

V_DEFINE_PROPERTIES(CalibrationPoseConfig)
(
    V_DEFINE_PROPERTY(pose, Pose(), "Pose of the object")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)

V_DEFINE_PROPERTIES(CalibrationPosesConfig)
(
    V_DEFINE_PROPERTY(poses, std::vector<CalibrationPoseConfig>(), "Poses of the object")
)
