#pragma once

#include <random>

#include "types.h"

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"

constexpr double AUTOMATIC_FILTER_SIZE = -1.;

//******************************************************************************
//******************************************************************************
DepthMap median_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE, bool permicroimage = false
);
void inplace_median_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE, bool permicroimage = false
);

DepthMap mean_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE
);
void inplace_mean_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc, 
	double size = AUTOMATIC_FILTER_SIZE
);

DepthMap minmax_filter_depth(const DepthMap& dm, double min, double max); 
void inplace_minmax_filter_depth(DepthMap& dm, double min, double max); 

//******************************************************************************
//******************************************************************************
DepthMap erosion_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc,
	double size = AUTOMATIC_FILTER_SIZE
);
void inplace_erosion_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc,
	double size = AUTOMATIC_FILTER_SIZE
); 

//******************************************************************************
//******************************************************************************
DepthMap bilateral_filter_depth(
	const DepthMap& dm, const PlenopticCamera& mfpc, 
	double sigmar = AUTOMATIC_FILTER_SIZE, double sigmad = AUTOMATIC_FILTER_SIZE, 
	bool permicroimage = false
);
void inplace_bilateral_filter_depth(
	DepthMap& dm, const PlenopticCamera& mfpc, 
	double sigmar = AUTOMATIC_FILTER_SIZE, double sigmad = AUTOMATIC_FILTER_SIZE, 
	bool permicroimage = false
); 

//******************************************************************************
//******************************************************************************
DepthMap consistency_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double threshold = AUTOMATIC_FILTER_SIZE);
void inplace_consistency_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double threshold = AUTOMATIC_FILTER_SIZE);
 
//******************************************************************************
//******************************************************************************
/* E(I,Z)= min I(Z) */ 
DepthMap morph_erosion_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_erosion_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 

/* D(I,Z)= max I(Z) */ 
DepthMap morph_dilation_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_dilation_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 

/* O(I,Z)= D(E(I,Z),Z) */ 
DepthMap morph_opening_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_opening_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 

/* C(I,Z)= E(D(I,Z),Z) */ 
DepthMap morph_closing_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_closing_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 

/* S(I,Z)= C(O(I,Z),Z) */ 
DepthMap morph_smoothing_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_smoothing_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 

/* DYT(I,Z) = 0.5 * (E(I,Z) + D(I,Z)) */
DepthMap morph_dyt_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_dyt_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 

/* TET(I,Z) = 0.5 * (O(I,Z) + C(I,Z)) */
DepthMap morph_tet_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_tet_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 

/* OCCO(I,Z) = 0.5 * (O(C(I,Z),Z) + C(O(I,Z),Z)) */
DepthMap morph_occo_filter_depth(const DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE);
void inplace_morph_occo_filter_depth(DepthMap& dm, const PlenopticCamera& mfpc, double size = AUTOMATIC_FILTER_SIZE); 


//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
//******************************************************************************
PointCloud minmax_filter_depth(const PointCloud& pc, double min, double max, Axis ax = Axis::Z); 
void inplace_minmax_filter_depth(PointCloud& pc, double min, double max, Axis ax = Axis::Z); 

PointCloud maxcount_filter_depth(const PointCloud& pc, std::size_t n); 
void inplace_maxcount_filter_depth(PointCloud& pc, std::size_t n); 

