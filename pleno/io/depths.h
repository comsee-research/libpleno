#pragma once

#include <map>

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/plane.h"

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"

#include "io/cfg/poses.h"

#include "io/cfg/depths.h"

//******************************************************************************
Pose 		read_mat(std::string path);
PointCloud 	read_pts(std::string path);
PointCloud	read_xyz(std::string path);

//******************************************************************************
std::map<Index, DepthMap> 		load(const DepthMapsConfig& config); //load virtual depth map from blade
std::map<Index, DepthMap> 		load_from_csv(std::string path, const PlenopticCamera& mfpc); //load metric depth map from pledge

std::map<Index, PointCloud> 	load(const PointCloudsConfig& config); //load pointclouds from blade
std::map<Index, PointCloud> 	load(const PTSsConfig& config); //load pts point cloud from leica
std::map<Index, PointCloud>		load(const XYZsConfig& config); //load xyz point cloud from raytrix

std::map<Index, Plane> 			load(const PlanesConfig& config); //load planes from blade

std::map<Index, Pose> 			load(const CalibrationPosesConfig& config); //load extrinsics poses from compote
std::map<Index, Pose> 			load(const MatsConfig& config); //load extrinsics poses from mat files

std::map<Index, Image> 			load(const CSADsConfig& config); //load csads from png files

std::map<Index, double> 		load_gt_dist(std::string path);
