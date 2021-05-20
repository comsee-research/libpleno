#pragma once

#include "io/archive.h"

V_DEFINE_PROPERTIES(DistortionConfig)
(
    V_DEFINE_PROPERTY(radial, 		(Eigen::Vector3d{0.0, 0.0, 0.0}), 	"Radial distortions coefficients (A_0, A_1 and A_2)")
    V_DEFINE_PROPERTY(tangential, 	(Eigen::Vector2d{0.0, 0.0}), 		"Tangential distortions coefficients (B_0 and B_1)")
    V_DEFINE_PROPERTY(depth, 		(Eigen::Vector3d{0.0, 0.0, 0.0}), 	"Depth distortions coefficients (D_0, D_1 and D_2)")
    V_DEFINE_PROPERTY(model, 		(std::uint16_t(0)), 				"DepthDistortionModel model: \
																			NO_DDM = 0, HEINZE_DDM = 1, ZELLER_DDM = 2, \
																			OFFSET_DDM = 3, LINEAR_DDM = 4, AFFINE_DDM = 5, QUADRATIC_DDM = 6"
	)																		
)
