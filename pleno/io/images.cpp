#include "images.h"

#include "io/printer.h"
#include "processing/imgproc/debayering.h"

void load(const std::vector<ImageWithInfoConfig>& cfgs, std::vector<ImageWithInfo>& images, bool debayer)
{
	images.reserve(cfgs.size());
	
	for(const auto& cfg : cfgs)
	{
		Image img, tmp = cv::imread(cfg.path(), cv::IMREAD_UNCHANGED);
		
		if (not debayer) debayering(tmp, img);
		else img = tmp;
		
		PRINT_DEBUG("Load image " << cfg.path());
		images.emplace_back(
			ImageWithInfo{ 
				std::move(img),
				cfg.fnumber(),
				cfg.frame()
			}
		);	
	}
}


void load(const ImageWithInfoConfig& cfg, ImageWithInfo& image, bool debayer)
{
	Image img, tmp = cv::imread(cfg.path(), cv::IMREAD_UNCHANGED);
	
	if (not debayer) debayering(tmp, img);
	else img = tmp;
	
	PRINT_DEBUG("Load image " << cfg.path());
	image = ImageWithInfo{std::move(img), cfg.fnumber(), cfg.frame()};
}
