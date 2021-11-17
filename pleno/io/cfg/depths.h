#pragma once

#include "io/archive.h"
#include <vector>

//******************************************************************************
V_DEFINE_PROPERTIES(XYZConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pointcloud to load (.xyz)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using XYZsConfig = std::vector<XYZConfig>;

//******************************************************************************
V_DEFINE_PROPERTIES(PTSConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pointcloud to load (.pts)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using PTSsConfig = std::vector<PTSConfig>;

//******************************************************************************
V_DEFINE_PROPERTIES(PointCloudConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pointcloud to load (.bin.gz)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using PointCloudsConfig = std::vector<PointCloudConfig>;

//******************************************************************************
V_DEFINE_PROPERTIES(DepthMapConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the depth map to load (.bin.gz)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using DepthMapsConfig = std::vector<DepthMapConfig>;

//******************************************************************************
V_DEFINE_PROPERTIES(CSVConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the depth map to load (.csv: k,l,d)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using CSVsConfig = std::vector<CSVConfig>;

//******************************************************************************
V_DEFINE_PROPERTIES(MatConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the pose to load (.mat)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using MatsConfig = std::vector<MatConfig>;

//******************************************************************************
V_DEFINE_PROPERTIES(PlaneConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the plane to load (.bin.gz)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using PlanesConfig = std::vector<PlaneConfig>;

//******************************************************************************
V_DEFINE_PROPERTIES(CSADConfig)
(
    V_DEFINE_PROPERTY(path, std::string(""), "Path to the central sub-aperture depth map (CSAD) to load (.png)")
    V_DEFINE_PROPERTY(frame, int(-1), "Frame index")
)
using CSADsConfig = std::vector<CSADConfig>;


//******************************************************************************
//******************************************************************************
//******************************************************************************
V_DEFINE_PROPERTIES(DepthsConfig)
(    
    V_DEFINE_PROPERTY(metric, bool(true), "Metric (true) or virtual (false) depth map")
    V_DEFINE_PROPERTY(coarse, bool(true), "Coarse (true) or refined (false) depth map")
    
    V_DEFINE_PROPERTY(xyzs, 		XYZsConfig(0), 			"Pointclouds (.xyz) configurations")
    V_DEFINE_PROPERTY(ptss, 		PTSsConfig(0), 			"Pointclouds (.pts) configurations")
    V_DEFINE_PROPERTY(pointclouds,	PointCloudsConfig(0), 	"Pointclouds (.bin.gz) configurations")
    
    V_DEFINE_PROPERTY(maps, 		DepthMapsConfig(0), 	"Depth Maps (.bin.gz) configurations")
    V_DEFINE_PROPERTY(csvs, 		CSVsConfig(0), 			"Depth Maps (.csv) configurations")
    
    V_DEFINE_PROPERTY(mats, 		MatsConfig(0), 			"Poses (.mat) configurations")
    
    V_DEFINE_PROPERTY(planes, 		PlanesConfig(0), 		"Planes (.bin.gz) configurations")
    
    V_DEFINE_PROPERTY(csads, 		CSADsConfig(0), 		"CSADs (.png) configurations")
    V_DEFINE_PROPERTY(refcsads, 	CSADsConfig(0), 		"Reference CSADs (.png) configurations")
)
