#pragma once

#include <utility>

#include "types.h"
#include "geometry/camera/plenoptic.h" //PlenopticCamera

#include "depthmap.h"
#include "pointcloud.h"

struct DepthMapImage {
public: 
	enum DepthInterpMethod : std::uint16_t { MIND = 0, MEDIAN = 1 };
	
private:
	static constexpr double minimal_resolvable_abs_depth 	= 2.;
	static constexpr double auto_dist_from_data 			= -1.;
	
public:
	Image depthmap; //CV_32F, to be save in .exr
	Image image;
	cv::Mat colormap;
		
public:
	DepthMapImage(const Image& dm, const cv::Mat& cm);
	DepthMapImage(const Image& dm, const Image& im, const cv::Mat& cm);
	
	DepthMapImage(
		const DepthMap& dm, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data
	);
	DepthMapImage(
		const PointCloud& pc, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data,
		DepthInterpMethod interpm = DepthInterpMethod::MEDIAN
	);
	
private:
	void compute_colormap(
		const DepthMap& dm, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data
	);
	void compute_colormap(
		const PointCloud& pc, const PlenopticCamera& model,
		double mind = auto_dist_from_data, double maxd = auto_dist_from_data
	);
	
	std::uint8_t scale_depth(double mind, double maxd, double d) const;	
	
	bool is_disparity_estimation_possible(const DepthMap& dm, const PlenopticCamera& model, double d) const;
	bool is_disparity_estimation_possible(const PointCloud& pc, const PlenopticCamera& model, double d) const;
};
