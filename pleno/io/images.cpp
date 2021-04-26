#include "images.h"

#include "io/printer.h"
#include "processing/imgproc/debayering.h"

void load(const std::vector<ImageWithInfoConfig>& cfgs, std::vector<ImageWithInfo>& images, bool debayered)
{
	images.reserve(cfgs.size());
	
	for(const auto& cfg : cfgs)
	{
		Image tmp = cv::imread(cfg.path(), cv::IMREAD_UNCHANGED);
		if (tmp.empty()) { PRINT_ERR("Cannot read image at: " << cfg.path()); continue; }
	
		Image img;	
		if (not debayered) debayering(tmp, img);
		else img = tmp;
		
		PRINT_DEBUG("Load image (" << img.cols << ", " << img.rows << ", " << img.channels() << ") from " << cfg.path());
		images.emplace_back(
			ImageWithInfo{ 
				std::move(img),
				cfg.fnumber(),
				cfg.frame()
			}
		);	
	}
}


void load(const ImageWithInfoConfig& cfg, ImageWithInfo& image, bool debayered)
{
	Image img, tmp = cv::imread(cfg.path(), cv::IMREAD_UNCHANGED);
	if (tmp.empty()) { PRINT_ERR("Cannot read image at: " << cfg.path()); return; }
	
	if (not debayered) debayering(tmp, img);
	else img = tmp;
	
	PRINT_DEBUG("Load image (" << img.cols << ", " << img.rows << ", " << img.channels() << ") from " << cfg.path());
	image = ImageWithInfo{std::move(img), cfg.fnumber(), cfg.frame()};
}
