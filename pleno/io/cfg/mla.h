#pragma once

#include <libv/core/serialization/serializable_properties.hpp> //V_DEFINE_PROPERTIES
#include <libv/core/serialization/contiguous_containers.hpp> //support for std::vector

#include "types.h"
#include "geometry/pose.h"

V_DEFINE_PROPERTIES(Mesh3DConfig)
(
    V_DEFINE_PROPERTY(pose, Pose(), "The pose of the grid")
    V_DEFINE_PROPERTY(height, (std::size_t(0)), "Number of rows in the array)")
    V_DEFINE_PROPERTY(width, (std::size_t(0)), "Number of cols in the array")
    V_DEFINE_PROPERTY(pitch, (P2D(P2D::Zero())), "Distances between two nodes (horizontally and vertically)")
    V_DEFINE_PROPERTY(geometry, (int(-1)), "Geometry of the grid (0: Orthogonal; 1: Hexagonal)")
    V_DEFINE_PROPERTY(orientation, (int(-1)), "Orientation of the grid (0: Horizontal; 1: Vertical)")
)

V_DEFINE_PROPERTIES(MLAConfig)
(
    V_DEFINE_PROPERTY(mesh, Mesh3DConfig(), "The grid representing the geometry of the MLA")
    V_DEFINE_PROPERTY(focal_lengths, std::vector<double>({0.,0.,0.}), "Focal lengths of the micro-lenses")
)

