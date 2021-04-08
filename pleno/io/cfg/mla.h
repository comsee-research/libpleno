#pragma once

#include "io/archive.h"

#include "types.h"
#include "geometry/pose.h"

V_DEFINE_PROPERTIES(Mesh3DConfig)
(
    V_DEFINE_PROPERTY(pose, Pose(), "The pose of the grid")
    V_DEFINE_PROPERTY(height, (std::size_t(0u)), "Number of rows in the array)")
    V_DEFINE_PROPERTY(width, (std::size_t(0u)), "Number of cols in the array")
    V_DEFINE_PROPERTY(pitch, (P2D{0.0, 0.0}), "Distances between two nodes (horizontally and vertically)")
    V_DEFINE_PROPERTY(geometry, (int(-1)), "Geometry of the grid (0: Orthogonal; 1: Hexagonal Rows Aligned; 2: Hexagonal Cols Aligned)")
)

V_DEFINE_PROPERTIES(MLAConfig)
(
    V_DEFINE_PROPERTY(mesh, Mesh3DConfig(), "The grid representing the geometry of the MLA")
    V_DEFINE_PROPERTY(focal_lengths, std::vector<double>{}, "Focal lengths of the micro-lenses")
)

