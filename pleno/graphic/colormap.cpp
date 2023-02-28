#include "colormap.h"

//Define colormaps
const Image PLENO_COLORMAP_VIRIDIS = [](void) -> Image {
	Image colormap = Image(1, 256, CV_8UC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = 
			cv::Vec3b{
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][2]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][1]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[i][0])
			};	
	return colormap;
}();

const Image PLENO_COLORMAP_VIRIDIS_INV = [](void) -> Image {
	Image colormap = Image(1, 256, CV_8UC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = 
			cv::Vec3b{
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[255 - i][2]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[255 - i][1]), 
				static_cast<std::uint8_t>(255. * viridis_srgb_floats[255 - i][0])
			};	
	return colormap;
}();

const Image PLENO_COLORMAP_VIRIDIS_32F = [](void) -> Image {
	Image colormap = Image(1, 256, CV_32FC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{viridis_srgb_floats[i][2], viridis_srgb_floats[i][1], viridis_srgb_floats[i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_VIRIDIS_32F_INV = [](void) -> Image {
	Image colormap = Image(1, 256, CV_32FC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{viridis_srgb_floats[255 - i][2], viridis_srgb_floats[255 - i][1], viridis_srgb_floats[255 - i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO = [](void) -> Image {
	Image colormap = Image(1, 256, CV_8UC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{turbo_srgb_bytes[i][2], turbo_srgb_bytes[i][1], turbo_srgb_bytes[i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO_INV = [](void) -> Image {
	Image colormap = Image(1, 256, CV_8UC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3b>(0,i) = cv::Vec3b{turbo_srgb_bytes[255 - i][2], turbo_srgb_bytes[255 - i][1], turbo_srgb_bytes[255 - i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO_32F = [](void) -> Image {
	Image colormap = Image(1, 256, CV_32FC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{turbo_srgb_floats[i][2], turbo_srgb_floats[i][1], turbo_srgb_floats[i][0]};	
	return colormap;
}();

const Image PLENO_COLORMAP_TURBO_32F_INV = [](void) -> Image {
	Image colormap = Image(1, 256, CV_32FC3);
	for(int i = 0; i < 256; ++i)
		colormap.at<cv::Vec3f>(0,i) = cv::Vec3f{turbo_srgb_floats[255 - i][2], turbo_srgb_floats[255 - i][1], turbo_srgb_floats[255 - i][0]};	
	return colormap;
}();

