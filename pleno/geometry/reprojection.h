#pragma once

#include "types.h"

#include "geometry/camera/models.h"
#include "geometry/object/checkerboard.h"

#include "geometry/observation.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel_t>
P2D reproject_miccenter(const CameraModel_t& model, const MICObservation& c)
{	
	//observation indexes in ML space
	const P3D ml_center = from_coordinate_system_of(
		model.mla().pose(), 
		model.mla().node(c.k, c.l)
	); //CAMERA
	const P3D optical_center = model.main_lens().pose().translation(); //CAMERA
	
	Ray3D ray;
	ray.config(optical_center, ml_center);
	
	const P3D projected_center = line_plane_intersection(model.sensor().planeInWorld(), ray); //CAMERA
	P2D center = to_coordinate_system_of(model.sensor().pose(), projected_center).head(2); //SENSOR
	center = model.sensor().metric2pxl(center); //IMAGE XY
	    	
	model.xy2uv(center); //IMAGE UV
	return center;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline 
P3D reproject_bapfeature(
	const MultiFocusPlenopticCamera& model, const Pose& pose, 
	const CheckerBoard& grid, const BAPObservation& observation
)
{
	//observation indexes in ML space
	const P3D p3d = grid.nodeInWorld(observation.cluster); // WORLD
	const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

	P3D prediction; //IMAGE UV
	if (model.project(p3d_cam, observation.k, observation.l, prediction))
		return prediction;
	else 
		return P3D{-1.0, -1.0, -1.0};
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename CameraModel_t, typename Observation_t>
P2D reproject_corner(
	const CameraModel_t& model, const Pose& pose, 
	const CheckerBoard& grid, const Observation_t& observation
)
{
	//observation indexes in ML space
	const P3D p3d = grid.nodeInWorld(observation.cluster); // WORLD
	const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

	P2D prediction; //IMAGE UV
	if (model.project(p3d_cam, observation.k, observation.l, prediction))
		return prediction;
	else 
		return P2D{-1.0, -1.0};
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline
double reproject_radius(
	const MultiFocusPlenopticCamera& model, const Pose& pose, 
	const CheckerBoard& grid, const BAPObservation& observation
)
{
	//observation indexes in ML space
	const P3D p3d = grid.nodeInWorld(observation.cluster); // WORLD
	const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

	double prediction; //IMAGE UV
	if (model.project(p3d_cam, observation.k, observation.l, prediction))
		return prediction;
	else 
		return -1.0;
}
