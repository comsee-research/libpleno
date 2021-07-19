#pragma once

#include "types.h"

#include "graphic/gui.h" 

#include "geometry/pose.h"
#include "geometry/ray.h"
#include "geometry/mesh.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/object/checkerboard.h"
#include "geometry/object/plate.h"
#include "geometry/object/constellation.h"

void viewer_3d(v::ViewerContext& v, const CheckerBoard& gm, double scale = 2.);

void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::CameraBody);
void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::ThinLens, double scale = 2.);
void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::MLA, double scale = 2.);
void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::Sensor, double scale = 2.);

// Drawing poses
void viewer_3d(v::ViewerContext&, const Pose& p, double = 1.0);
void viewer_3d(v::ViewerContext&, const Poses& ps, double = 1.0);

// Drawing some rays
void viewer_3d(v::ViewerContext&, const Ray3D& r);
void viewer_3d(v::ViewerContext&, const Rays3D& rays);

// Drawing some points
void viewer_3d(v::ViewerContext&, const P3D& p, double = 1.0);
void viewer_3d(v::ViewerContext&, const P3DS& ps, double = 1.0);
void viewer_3d(v::ViewerContext& v, const PointsConstellation& points, double scale = 5.0);

// Displaying a GridMesh3D
void viewer_3d(v::ViewerContext&, const GridMesh3D& gm);

// Displaying a plate in 3D
void viewer_3d(v::ViewerContext&, const Plate& plate);
