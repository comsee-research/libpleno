#pragma once

#include <libv/core/serialization/serializable_properties.hpp>
#include <libv/core/serialization/eigen.hpp>

V_DEFINE_PROPERTIES(DistortionConfig)
(
    V_DEFINE_PROPERTY(radial, Eigen::Vector3d(-1.0, -1.0, -1.0), "")
    V_DEFINE_PROPERTY(tangential, Eigen::Vector2d(-1.0, -1.0), "")
)
