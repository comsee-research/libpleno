#include "trim.h"

////////////////////////////////////////////////////////////////////////////////
/*
	Image must be in 8-bits format
*/
void trim(Image& img, double r, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2.;
	const double centery = height / 2.;
	
	uchar * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<uchar>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) < r+tolerance) continue;
			
			pixel[x] = 0;
		}
	}
}
/*
	Image must be in 8-bits format
*/
void trim_binarize(Image& img, double radius, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2.;
	const double centery = height / 2.;
	
	uchar * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<uchar>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) < radius+tolerance) //in
			{
				pixel[x] = 255; 
			}
			else //out
			{	
				pixel[x] = 0;	
			}			
		}
	}
}
/*
	Image must be in 32-bits format
*/
void trim_float(Image& img, double radius, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2. - 0.5;
	const double centery = height / 2. - 0.5;
	
	float * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<float>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) >= radius+tolerance) //out
			{
				pixel[x] = 0.f;	
			}			
		}
	}
}
/*
	Image must be in 32-bits format
*/
void trim_float_binarize(Image& img, double radius, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2. - 0.5;
	const double centery = height / 2. - 0.5;
	
	float * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<float>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) >= radius+tolerance) //out
			{
				pixel[x] = 0.f;	
			}	
			else //in
			{
				pixel[x] = 1.f;
			}		
		}
	}
}
/*
	Image must be in 64-bits format
*/
void trim_double(Image& img, double radius, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2. - 0.5;
	const double centery = height / 2. - 0.5;
	
	double * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<double>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) >= radius+tolerance) //out
			{
				pixel[x] = 0.;	
			}			
		}
	}
}
/*
	Image must be in 64-bits format
*/
void trim_double_binarize(Image& img, double radius, double tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const double centerx = width / 2. - 0.5;
	const double centery = height / 2. - 0.5;
	
	double * pixel;
	for(int y = 0 ; y < height ; ++y) //for each row
	{
		pixel = img.ptr<double>(y);
		for(int x = 0 ; x < width ; ++x) //for each column
		{
			if(std::hypot(x-centerx, y-centery) >= radius+tolerance) //out
			{
				pixel[x] = 0.;	
			}	
			else //in
			{
				pixel[x] = 1.;
			}		
		}
	}
}

