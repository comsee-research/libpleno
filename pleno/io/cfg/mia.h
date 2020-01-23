#pragma once

#include <libv/core/serialization/serializable_properties.hpp>

#include "types.h"
#include "geometry/pose.h"

V_DEFINE_PROPERTIES(Mesh2DConfig)
(
    V_DEFINE_PROPERTY(pose, Pose2D(), "The pose of the grid")
    V_DEFINE_PROPERTY(height, (std::size_t(0)), "Number of rows in the array)")
    V_DEFINE_PROPERTY(width, (std::size_t(0)), "Number of cols in the array")
    V_DEFINE_PROPERTY(pitch, (P2D(P2D::Zero())), "Distances between two nodes (horizontally and vertically)")
    V_DEFINE_PROPERTY(geometry, (int(-1)), "Geometry of the grid (0: Orthogonal; 1: Hexagonal Rows Aligned; 2: Hexagonal Cols Aligned)")
)

V_DEFINE_PROPERTIES(MIAConfig)
(
    V_DEFINE_PROPERTY(mesh, Mesh2DConfig(), "The grid representing the geometry of the MIA")
)

