#pragma once

#include "types.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/fast_math.hpp>

#include "geometry/plane.h"


////////////////////////////////////////////////////////////////////////////////
// Micro-image estimation of properties
////////////////////////////////////////////////////////////////////////////////
GaussianCoefficients 
estimation_gaussian_moments(
	const Image& img, 
	bool assumed_circular = false //if set to true, simply assume that sigma2x=sigma2y
);

GaussianCoefficients 
estimation_gaussian_least_squares(
	const Image& img, 
	const GaussianCoefficients& init, //initial values of mean 
	bool truncated = false, //take into account saturated pixel or not
	const unsigned int thr = 200 //threshold above which to trunc the saturated pixels
);

CircleCoefficients 
estimation_min_enclosing_circle(
	Image& img
);

////////////////////////////////////////////////////////////////////////////////
// Estimation on data points
////////////////////////////////////////////////////////////////////////////////
std::vector<LineCoefficients>
estimation_lines_with_slope_constraint_least_squares(
	const std::vector<std::vector<P2D>>& data
);

LineCoefficients
estimation_line_fitting(
	const std::vector<P2D>& data, 
	cv::DistanceTypes dist= cv::DIST_L1 //dist in cv:: { DIST_USER, DIST_L1, DIST_L2, DIST_C, DIST_L12, DIST_FAIR, DIST_WELSCH, DIST_HUBER }
);

////////////////////////////////////////////////////////////////////////////////
// Estimation on 3D points
////////////////////////////////////////////////////////////////////////////////
Plane 
estimation_plane_fitting(
	const P3DS& pts
);

Plane
estimation_plane_ransac(
	const P3DS& pts,
	double threshold, //Threshold value to determine data points that are fit well by model.
	std::size_t n = 50ul, //Minimum number of data points required to estimate model parameters.
	std::size_t k = 100ul, //Maximum number of iterations allowed in the algorithm.
	std::size_t d = 50ul //Number of close data points required to assert that a model fits well to data.
);


