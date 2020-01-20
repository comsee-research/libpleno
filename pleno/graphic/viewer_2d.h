#pragma once

#include "types.h"

#include "graphic/gui.h" 

#include "geometry/disk.h"
#include "geometry/ray.h"
#include "geometry/mesh.h"

// Wrapper libv for opencv mat
void libv_wrapper(v::ViewerContext& v, const Image& image, int u_position = 0, int v_position = 0);

// Displaying images
void viewer_2d(v::ViewerContext& v, const Image& image, int u_position = 0, int v_position = 0);

// Displaying points
void viewer_2d(v::ViewerContext& v, const std::vector<cv::Point2f>& points);
void viewer_2d(v::ViewerContext& v, const P2D& point);
void viewer_2d(v::ViewerContext& v, const P2DS& points);

// Displaying rays
void viewer_2d(v::ViewerContext& v, const Ray2D& ray, double = 10.0);

// Displaying disk
void viewer_2d(v::ViewerContext& v, const Disk& d);

// Displaying a GridMesh2D
void viewer_2d(v::ViewerContext& v, const GridMesh2D& gm);
