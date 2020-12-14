#include "access.h"

////////////////////////////////////////////////////////////////////////////////
std::size_t size(const Image& i) 	{ return i.cols * i.rows; }
std::size_t width(const Image& i)	{ return i.cols; }
std::size_t height(const Image& i) 	{ return i.rows; }

////////////////////////////////////////////////////////////////////////////////
/* 
	extract region of interest around the point (X,Y) of size (roiw x roih)
	when the roi contains pixels outside of the images, the roi is reduce, 
	and the new coordinates of the top-left corner are save in the (X,Y) variables
 */ 
Image extract_roi(const Image&img, double& X, double& Y, int roiw, int roih)
{
	//crop image aroud the center
	const int cx = static_cast<int>(X - roiw/2);
	const int cy = static_cast<int>(Y - roih/2);
	   	
	const int x = (cx < 0 ? 0 : (cx > img.cols ?  img.cols-roiw/2 : cx)); X = x;
	const int y = (cy < 0 ? 0 : (cy > img.rows ?  img.rows-roih/2 : cy)); Y = y;
	
	const int eroiw = (cx < 0) ? (roiw - std::abs(cx) +1) : ((cx > (img.cols - roiw)) ? (img.cols - x) : roiw);
	const int eroih = (cy < 0) ? (roih - std::abs(cy) +1) : ((cy > (img.rows - roih)) ? (img.rows - y) : roih);

	return img(cv::Rect{x,y, eroiw, eroih});
}
