#pragma once

#include "types.h"

#include "geometry/camera/models.h"
#include "geometry/object/checkerboard.h"

#include "geometry/observation.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline P2D reproject_miccenter(const PlenopticCamera& model, const MICObservation& c)
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
	P2D center = to_coordinate_system_of(model.sensor().pose(), projected_center).head<2>(); //SENSOR
	center = model.sensor().metric2pxl(center); //IMAGE XY
	    	
	model.xy2uv(center); //IMAGE UV
	return center;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline 
P3D reproject_bapfeature(
	const PlenopticCamera& model, const Pose& pose, 
	const CheckerBoard& grid, const BAPObservation& observation, bool check = false
)
{
	DEBUG_ASSERT((model.I()>0u), "Can't reproject radius in BAP feature.");
	
	//observation indexes in ML space
	const P3D p3d = grid.nodeInWorld(observation.cluster); // WORLD
	const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

	P3D prediction; //IMAGE UV
	if (model.project(p3d_cam, observation.k, observation.l, prediction) or not check)
	{
		return prediction;
	}
	else 
	{
		PRINT_WARN("Observation not reprojected ("<<prediction<<")");
		return prediction; //P3D{-1.0, -1.0, -1.0};
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observation>
P2D reproject_corner(
	const PlenopticCamera& model, const Pose& pose, 
	const CheckerBoard& grid, const Observation& observation, bool check = false
)
{
	//observation indexes in ML space
	const P3D p3d = grid.nodeInWorld(observation.cluster); // WORLD
	const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

	P2D prediction; //IMAGE UV
	if (model.project(p3d_cam, observation.k, observation.l, prediction) or not check)
	{
		return prediction;
	}
	else 
	{
		PRINT_WARN("Observation not reprojected ("<<prediction<<")");
		return P2D{-1.0, -1.0};
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline
double reproject_radius(
	const PlenopticCamera& model, const Pose& pose, 
	const CheckerBoard& grid, const BAPObservation& observation
)
{
	DEBUG_ASSERT((model.I()>0u), "Can't reproject radius in BAP feature.");
	
	//observation indexes in ML space
	const P3D p3d = grid.nodeInWorld(observation.cluster); // WORLD
	const P3D p3d_cam = to_coordinate_system_of(pose, p3d); // CAMERA

	double prediction; //IMAGE UV
	if (model.project(p3d_cam, observation.k, observation.l, prediction))
	{
		return prediction;
	}
	else 
	{
		return 0.0;
	}
}

