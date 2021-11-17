#include "depthimage.h"

#include <opencv2/imgproc.hpp>

#include "graphic/colormap.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
DepthMapImage::DepthMapImage(const Image& dm, const cv::Mat& cm)
: depthmap{dm}, colormap{cm}
{
	cv::LUT(depthmap, colormap, image);
}

DepthMapImage::DepthMapImage(const Image& dm, const Image& im, const cv::Mat& cm)
: depthmap{dm}, image{im}, colormap{cm}
{

}

DepthMapImage::DepthMapImage(const DepthMap& dm, const PlenopticCamera& model, double mind, double maxd)
{		
	if (mind == auto_dist_from_data) mind = dm.min_depth();
	if (maxd == auto_dist_from_data) maxd = dm.max_depth();	
	
	//get colormap
	compute_colormap(dm, model, mind, maxd);
	
	//get image
	const std::size_t img_width 	= model.sensor().width();
	const std::size_t img_height 	= model.sensor().height();
	
	const std::uint8_t d0 	= scale_depth(mind, maxd, DepthInfo::NO_DEPTH);
	const float fd0 		= static_cast<float>(DepthInfo::NO_DEPTH);

	depthmap 		= Image(img_height, img_width, CV_32FC3, cv::Scalar(fd0,fd0,fd0));
	Image sdepthmap = Image(img_height, img_width, CV_8UC3, cv::Scalar(d0,d0,d0));
	
	for(std::size_t k = 0; k < dm.width(); ++k)
	{
		for(std::size_t l = 0; l < dm.height(); ++l)
		{
			const double depth 		= dm.depth(k,l) * (dm.is_valid_depth(dm.depth(k,l)));	
			const float fdepth 		= static_cast<float>(depth);	
			const std::uint8_t d 	= scale_depth(mind, maxd, depth);	
				
			if (dm.is_coarse_map())
			{
				const auto center 	= model.mia().nodeInWorld(k, l);
				const double radius = model.mia().radius();			
				
				cv::circle(
					depthmap, cv::Point(center[0], center[1]), radius,
					cv::Scalar(fdepth,fdepth,fdepth), CV_FILLED
				);
				
				cv::circle(
					sdepthmap, cv::Point(center[0], center[1]), radius,
					cv::Scalar(d,d,d), CV_FILLED
				);	
			}
			else if (dm.is_refined_map())
			{
				depthmap.at<cv::Vec3f>(l, k) 	= cv::Vec3f{fdepth, fdepth, fdepth};	//(row,col) access
				sdepthmap.at<cv::Vec3b>(l, k) 	= cv::Vec3b{d, d, d};	//(row,col) access
			}
		}
	}
			
	cv::LUT(sdepthmap, colormap, image);			
}


DepthMapImage::DepthMapImage(
	const PointCloud& pc, const PlenopticCamera& model, 
	double mind, double maxd, DepthInterpMethod interpm
)
{	
	//Configure monocular camera
	Sensor film = Sensor{model.sensor().width() / 4, model.sensor().height() / 4, model.sensor().scale() * 4};
	film.pose() = model.sensor().pose();
	film.pose().translation().z() = - model.focal();
	
	const PinholeCamera monocular{model.focal(), film};
	
	const auto [mind_, maxd_] = pc.minmax();  DEBUG_VAR(mind_); DEBUG_VAR(maxd_);
	if (mind == auto_dist_from_data) mind = mind_;
	if (maxd == auto_dist_from_data) maxd = maxd_;
	
	//get colormap
	compute_colormap(pc, model, mind, maxd);
	
	//get image
	const std::size_t img_width 	= film.width();
	const std::size_t img_height 	= film.height();
	
	const std::uint8_t d0 	= scale_depth(mind, maxd, DepthInfo::NO_DEPTH);
	const float fd0 		= static_cast<float>(DepthInfo::NO_DEPTH);

	depthmap		= Image(img_height, img_width, CV_32FC3, cv::Scalar(fd0,fd0,fd0));
	Image sdepthmap = Image(img_height, img_width, CV_8UC3, cv::Scalar(d0,d0,d0));
	
	using ZBufferContainer = Eigen::Matrix<std::vector<double>, Eigen::Dynamic /* row */, Eigen::Dynamic /* col */>;
	
	ZBufferContainer mzbuffer{img_height, img_width};
	auto zbuffer = [&](int u, int v) -> std::vector<double>& { return mzbuffer(v, u); }; 
	
	#pragma omp parallel for
	for(int v = 0; v < int(img_height); ++v)
		for(int u = 0; u < int(img_width); ++u)
			zbuffer(u, v).reserve(20);
	
	#pragma omp parallel for
	//for each point
	for (int j = 0; j < int(pc.size()); ++j)
	{
		const P3D& p = pc.feature(j);
		
		//project point
		P2D pixel;
		if (monocular.project(p, pixel))
		{
			#pragma omp critical
			mzbuffer(pixel(1), pixel(0)).emplace_back(p.z());	
		}
	}
	
	#pragma omp parallel for
	for(int v = 0; v < int(img_height); ++v)
	{
		for(int u = 0; u < int(img_width); ++u)
		{
			if (interpm == DepthInterpMethod::MEDIAN)
			{
				if (std::size_t sz = zbuffer(u, v).size(); sz > 1) 
				{
					std::nth_element(std::begin(zbuffer(u, v)), std::begin(zbuffer(u, v)) + sz / 2, std::end(zbuffer(u, v)));
					const double depth = zbuffer(u, v)[sz / 2];
					const float fdepth = static_cast<float>(depth);	
								
					//assign depth
					depthmap.at<cv::Vec3f>(v, u) 	= cv::Vec3f{fdepth, fdepth, fdepth};	//(row,col) access
					
					const std::uint8_t d 			= scale_depth(mind, maxd, depth);	
					sdepthmap.at<cv::Vec3b>(v, u) 	= cv::Vec3b{d, d, d};	//(row,col) access
				}
			}
			else if (interpm == DepthInterpMethod::MIND)
			{
				if (std::size_t sz = zbuffer(u, v).size(); sz > 0) 
				{
					const double depth = *std::min_element(std::begin(zbuffer(u, v)), std::end(zbuffer(u, v)));	
					const float fdepth = static_cast<float>(depth);	
												
					//assign depth
					depthmap.at<cv::Vec3f>(v, u) 	= cv::Vec3f{fdepth, fdepth, fdepth};	//(row,col) access
					
					const std::uint8_t d 			= scale_depth(mind, maxd, depth);	
					sdepthmap.at<cv::Vec3b>(v, u) 	= cv::Vec3b{d, d, d};	//(row,col) access
				}	
			}
		}
	}
	
	cv::LUT(sdepthmap, colormap, image);	
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void DepthMapImage::compute_colormap(const DepthMap& dm, const PlenopticCamera& model, double mind, double maxd)
{
	if (mind == auto_dist_from_data) mind = dm.min_depth();
	if (maxd == auto_dist_from_data) maxd = dm.max_depth();	

#if 0
 	cv::Mat lut(1, 256, CV_8UC1);
	uchar *ptr = lut.ptr<uchar>(0);
	for(int i = 0; i < 256; ++i) ptr[i] = i;
	
	cv::applyColorMap(lut, colormap, cv::COLORMAP_JET);
#else
	colormap = PLENO_COLORMAP_VIRIDIS_INV; //PLENO_COLORMAP_TURBO_INV; //
	constexpr std::uint8_t d0 = 0;//255; //127; // 
#endif
		
	if(dm.is_metric_depth() or mind > minimal_resolvable_abs_depth) 
	{
		colormap.at<cv::Vec3b>(0,0) = cv::Vec3b{d0,d0,d0};
	}
	
	for(int i = 0; i < 256; ++i)
	{
		if(not is_disparity_estimation_possible(dm, model, i * (maxd - mind) / 255. + mind))
			colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{d0,d0,d0};		
	}
}

void DepthMapImage::compute_colormap(const PointCloud& pc, const PlenopticCamera& model, double mind, double maxd)
{
	const auto [mind_, maxd_] = pc.minmax();
	if (mind == auto_dist_from_data) mind = mind_;
	if (maxd == auto_dist_from_data) maxd = maxd_;
 	
#if 0
 	cv::Mat lut(1, 256, CV_8UC1);
	uchar *ptr = lut.ptr<uchar>(0);
	for(int i = 0; i < 256; ++i) ptr[i] = i;
	
	cv::applyColorMap(lut, colormap, cv::COLORMAP_JET);
#else
	colormap = PLENO_COLORMAP_VIRIDIS_INV; //PLENO_COLORMAP_TURBO_INV; //
	constexpr std::uint8_t d0 = 0;//255; //127; // 
#endif
			
	if(model.obj2v(maxd) > minimal_resolvable_abs_depth) 
	{
		colormap.at<cv::Vec3b>(0,0) = cv::Vec3b{d0,d0,d0};
	}
	
	for(int i = 0; i < 256; ++i)
	{
		if(not is_disparity_estimation_possible(pc, model, i * (maxd - mind) / 255. + mind))
			colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{d0,d0,d0};		
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::uint8_t DepthMapImage::scale_depth(double mind, double maxd, double d) const
{
	return std::min(255, std::max(0,
		static_cast<int>(255 * (d - mind) / (maxd - mind))
	));
}
	
//******************************************************************************
//******************************************************************************
//******************************************************************************
bool DepthMapImage::is_disparity_estimation_possible(const DepthMap& dm, const PlenopticCamera& model, double d) const
{
	if(dm.is_metric_depth()) d = model.obj2v(d); //convert to virtual depth
	return (std::fabs(d) > minimal_resolvable_abs_depth);	
}

bool DepthMapImage::is_disparity_estimation_possible(const PointCloud& /*pc*/, const PlenopticCamera& model, double d) const
{
	d = model.obj2v(d); //convert to virtual depth
	return (std::fabs(d) > minimal_resolvable_abs_depth);	
}
