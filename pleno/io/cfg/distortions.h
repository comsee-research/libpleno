#pragma once

#include "io/archive.h"

V_DEFINE_PROPERTIES(DistortionConfig)
(
    V_DEFINE_PROPERTY(radial, (Eigen::Vector3d{0.0, 0.0, 0.0}), "Radial distortions coefficients (A_0, A_1 and A_2)")
    V_DEFINE_PROPERTY(tangential, (Eigen::Vector2d{0.0, 0.0}), "Tangential distortions coefficients (B_0 and B_1)")
)
