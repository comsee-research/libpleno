/*
 * estimation.cpp
 * This file is part of multifocus_calibration
 *
 * Copyright (C) 2019 - Mathieu Labussiere
 *
 * multifocus_calibration is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * multifocus_calibration is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with multifocus_calibration. If not, see <http://www.gnu.org/licenses/>.
 */
 	 
#include "estimation.h"

#include "graphic/gui.h"
#include "io/printer.h"
#include "processing/improcess.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/fast_math.hpp>

GaussianCoefficients 
estimation_gaussian_moments(
	const Image& img, bool assumed_circular
)
{
	//GAUSSIAN FITTING USING MOMENTS	
	cv::Moments ms = cv::moments(img);
	
	const double sigma2x = ms.mu20 / ms.m00;
	const double sigma2y = ms.mu02 / ms.m00;
	const double sigmaxy = ms.mu11 / ms.m00;
	
	const double meanx 	= ms.m10  / ms.m00;
	const double meany  = ms.m01  / ms.m00;
	
	const double lambdamax  = (sigma2x + sigma2y) / 2. + std::sqrt(4. * sigmaxy * sigmaxy + (sigma2x - sigma2y) * (sigma2x - sigma2y)) / 2. ;
	//const double lambdamin  = (sigma2x + sigma2y) / 2. - std::sqrt(4. * sigmaxy * sigmaxy + (sigma2x - sigma2y) * (sigma2x - sigma2y)) / 2. ;
	
	const double sigma = assumed_circular ? std::sqrt((sigma2x+sigma2y)/2.) : std::sqrt(lambdamax);
	
	if(not assumed_circular)
		PRINT_DEBUG("Gaussian moments fitting =  exp(-((x-"<<meanx<<")²/2*"<<sigma2x<<") + ((y-"<<meany<<")²/2*"<<sigma2y<<"))");
	else
		PRINT_DEBUG("Gaussian moments fitting =  exp(-((x-"<<meanx<<")² + (y-"<<meany<<")²) /2*"<<sigma*sigma<<"))");
	
	return {meanx, meany, sigma, 1.};
}

GaussianCoefficients 
estimation_gaussian_least_squares(
	const Image& img, const GaussianCoefficients& init, bool truncated, const unsigned int thr
)
{//LEAST SQUARE ESTIMATION OF SIGMA
	using A_t = Eigen::Matrix<float, Eigen::Dynamic, 2>;
	using B_t = Eigen::Matrix<float, Eigen::Dynamic, 1>;
	using X_t = Eigen::Matrix<float, 2, 1>;
	
	const auto& [meanx, meany, s, a] = init;
	const std::size_t eroiw = img.cols;
	const std::size_t eroih = img.rows;
	
	//compute size
	std::size_t size = eroiw * eroih;
	
	A_t A = A_t::Zero(size,2);
	B_t B = B_t::Zero(size);
	
	const uchar * pixel;
	for(std::size_t y = 0 ; y < eroih ; ++y) //for each row
	{
		pixel = img.ptr<uchar>(y);
		for(std::size_t x = 0 ; x < eroiw ; ++x) //for each column
		{
			const int i = y*eroih+x;
			
			if(truncated and pixel[x] >= thr) continue;
			
			A(i, 0) = (x-meanx)*(x-meanx) + (y-meany)*(y-meany);
			A(i, 1) = 1;
			B(i) 	= std::log(pixel[x]+1e-7);
		}
	}
	
	X_t X = A.colPivHouseholderQr().solve(B);
	//DEBUG_VAR(X);  

	const float sigma2 	= - 1. / (2. * X(0));
	const float sigma	= std::sqrt(sigma2);
	const float alpha	= std::exp(X(1));
	
	PRINT_DEBUG("Sigma estimation =  "<<alpha<<" * exp( -( (x - "<<meanx<<")² + (y - "<<meany<<")² / 2 * "<<sigma2<<"))");
	
	return {meanx, meany, sigma, alpha};
}

CircleCoefficients 
estimation_min_enclosing_circle(
	Image& img
)
{ //MIN ENCLOSING CIRCLE ESTIMATION
	const std::size_t eroiw = img.cols;
	const std::size_t eroih = img.rows;

	//detect shapes
	std::vector<std::vector<cv::Point>> contours = detect_shapes(img);
	const std::size_t nb_contours = contours.size();
	std::vector<std::vector<cv::Point>> contours_polygon(nb_contours);
	
	std::vector<float> radius(nb_contours);
	std::vector<cv::Point2f> center(nb_contours);

	for (std::size_t c = 0; c < nb_contours; ++c)
	{
		cv::approxPolyDP(cv::Mat(contours[c]), contours_polygon[c], 3, true);
		cv::minEnclosingCircle(cv::Mat(contours_polygon[c]), center[c], radius[c]);
	}
	
	//center the index of the closest detected to the observation (eroiw/2,eroih/2) in roi coordinates
	int index = -1; 
	float mindist = eroiw + eroih;
	for(std::size_t i = 0; i < center.size() ; ++i)
	{
		const auto &c = center[i];
		const float dist = std::hypot(eroiw/2 - c.x, eroih/2 - c.y);
		if(dist < mindist)
		{
			mindist = dist;
			index = i;
		}
	}
	
	if(index != -1)
		return {static_cast<double>(center[index].x), static_cast<double>(center[index].y), static_cast<double>(radius[index])};
	else
		return {0.,0.,0.};
}

std::vector<LineCoefficients>
estimation_lines_with_slope_constraint_least_squares(
	const std::vector<std::vector<P2D>>& data
)
{
	const std::size_t I = data.size();
	
	std::vector<LineCoefficients> coefs;
	coefs.reserve(I);
	
//LINEAR REGRESSION USING CONSTRAINTS ON SLOPE	
	using A_t = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
	using B_t = Eigen::Matrix<float, Eigen::Dynamic, 1>;
	using X_t = Eigen::Matrix<float, Eigen::Dynamic, 1>;
	
	//compute size
	std::size_t size = 0;
	for(const auto&points : data) size+=points.size();
	
	A_t A = A_t::Zero(size, I+1);
	B_t B = B_t::Zero(size);
	
	int i=1,j=0;	
	for(const auto&points : data)
    {
    	for(const auto&p : points)
    	{
    		A(j,0) 	= p[0];
    		A(j,i)	= 1;
    		B(j) 	= p[1];
    	
    		++j;
    	}
    	++i;    
    }
    
    X_t X = A.colPivHouseholderQr().solve(B);
	DEBUG_VAR(X);  
	
	for(std::size_t i = 0; i < I; ++i)
		coefs.emplace_back(LineCoefficients{X(0), X(i+1)});
	
	return coefs;
}

LineCoefficients
estimation_line_fitting(
	const std::vector<P2D>& data, cv::DistanceTypes dist
)
{//LINE FITTING USING OPENCV
	std::vector<cv::Point2f> points;
	points.reserve(data.size());
	
	for(const auto&point : data)
    	points.emplace_back(point[0], point[1]);
	
	cv::Vec4f line;
	// find the optimal line
	cv::fitLine(points, line, dist, 0, 0.0001, 0.0001);
	
	const float m = line[1] / line[0];
	const float c = line[3] - m * line[2];
	
	PRINT_DEBUG("Line: y = "<< m << " * x + " << c);	
	
	return {m,c};	
}
