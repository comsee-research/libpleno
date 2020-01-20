#pragma once

#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/core/serialization/contiguous_containers.hpp> //support for std::vector

#include "types.h"
#include "geometry/pose.h"


V_DEFINE_PROPERTIES(CubeConfig)
(
    V_DEFINE_PROPERTY(pose, Pose(), "Pose of the object")
    V_DEFINE_PROPERTY(lph, double(-1), "Edge length")
)

V_DEFINE_PROPERTIES(CheckerboardConfig)
(
    V_DEFINE_PROPERTY(pose, Pose(), "Pose of the object")
    V_DEFINE_PROPERTY(lph, double(-1.0), "")
    V_DEFINE_PROPERTY(x_grid, int(-1), "")
    V_DEFINE_PROPERTY(y_grid, int(-1), "")
)

V_DEFINE_PROPERTIES(PlateConfig)
(
    V_DEFINE_PROPERTY(pose, Pose(), "Pose of the object")
    V_DEFINE_PROPERTY(lph, double(-1), "")
    V_DEFINE_PROPERTY(texture_path, std::string(), "the path of the texture file (png)")
    V_DEFINE_PROPERTY(x_grid, int(-1), "")
    V_DEFINE_PROPERTY(y_grid, int(-1), "")
)

V_DEFINE_PROPERTIES(SceneConfig)
(
    V_DEFINE_PROPERTY(checkerboards, std::vector<CheckerboardConfig>(), 0)
    V_DEFINE_PROPERTY(cubes, std::vector<CubeConfig>(), 0)
    V_DEFINE_PROPERTY(plates, std::vector<PlateConfig>(), 0)
)

