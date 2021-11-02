#include "display.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/fast_math.hpp>

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"

#include "processing/tools/stats.h"

////////////////////////////////////////////////////////////////////////////////
// DISPLAYS - Histograms
////////////////////////////////////////////////////////////////////////////////
void hist_data(const std::vector<double>& data, Image& dst, double min, double max, int binSize, int height, int nbbin)
{		
	const double step = (max - min) / nbbin;
	std::vector<int> hist(nbbin);
	for(const auto&d : data)
	{
		if(d<min or d>max) continue;
		int i =  (d - min) / step;
		hist[i] +=1;	
	}

    int max_value = *std::max_element(hist.begin(), hist.end());
    int rows = 0;
    int cols = 0;
    double scale = 1;
    if (height == 0) {
        rows = max_value + 10;
    }
    else {
        rows = height; 
        scale = double(height) / (max_value + 10);
    }
    cols = hist.size() * binSize;
    dst = Image::zeros(rows, cols, CV_8UC3);
    for (std::size_t i = 0; i < hist.size(); ++i)
    {
        const int h = rows - int(scale * hist[i]);
        cv::rectangle(dst, cv::Point(i*binSize, h), cv::Point((i + 1)*binSize - 1, rows), (i % 2) ? cv::Scalar(0, 100, 255) : cv::Scalar(0, 0, 255), cv::FILLED);
    }

	const double rmean = mean(data);
	const double rmedian = median(data);
	
	const double col_mean = (rmean - min) / step * binSize;
	const double col_median = (rmedian - min) / step * binSize; 
	
	cv::line(dst, cv::Point(col_mean, 20), cv::Point(col_mean, rows), cv::Scalar(255,0,0));
	cv::line(dst, cv::Point(col_median, 10), cv::Point(col_median, rows), cv::Scalar(0,255,0));
	
	cv::putText(dst, 
            "Mean= " + std::to_string(rmean),
            cv::Point(col_mean+5, 20), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.5, // Scale. 2.0 = 2x bigger
            cv::Scalar(255,0,0), // BGR Color
            1 // Line Thickness (Optional)
	); 
	cv::putText(dst, 
            "Median= " + std::to_string(rmedian),
            cv::Point(col_median+5,10), // Coordinates
            cv::FONT_HERSHEY_COMPLEX_SMALL, // Font
            0.5, // Scale. 2.0 = 2x bigger
            cv::Scalar(0,255,0), // BGR Color
            1 // Line Thickness (Optional)
	); 
}

void hist_radii(const std::vector<MicroImage>& data)
{
	const std::size_t I = [&data]{
		int i=0;
		for(const auto& mi : data) if(mi.type > i) i = mi.type;
		return std::size_t(i+1);
	}();
	DEBUG_VAR(I);
	
	std::vector<std::vector<double>> radii(I);
    for(auto& r : radii) r.reserve(data.size());
	
    for(const auto&mi : data)
    {
    	radii[mi.type].emplace_back(mi.radius);
    }
    
    for(auto& r : radii) r.shrink_to_fit();
    
/////////////////////////////////////Compute histograms///////////////////////////////////////////////////   
    const auto& [min, max] = [&data]() -> std::pair<double, double> {
		double min=1e12, max=-1e12;
		for(const auto&mi : data) {
			const double d = mi.radius;
			if(d < min) min = d;
			if(d > max) max = d;
		}
		return {std::floor(min), std::ceil(max)};
	}(); 
	
    Images hist(I);
    for(std::size_t i=0; i<I; ++i)
    { 
    	hist_data(radii[i], hist[i], min, max, 3, 200);
   	}
   	
	Image histo = hist[0];
	for(std::size_t i=1; i<I; ++i) 
	{
    	vconcat(histo, hist[i], histo);
    }
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("radii_histo"), histo); 
}

////////////////////////////////////////////////////////////////////////////////
// DISPLAYS - data
////////////////////////////////////////////////////////////////////////////////
void display_data(const std::vector<P2D>& pts, const LineCoefficients& coefs, Image& out)
{
	const std::size_t rows = 500;
	const std::size_t cols = 600;
	
	const double xmax = 0.3;
	const double xmin = 0.0;
	const double ymin = 0.01; //FIXME
	const double ymax = 0.1;
	
	const auto & [m,c] = coefs;
	const double cscaled = rows * (1. - ((c - ymin) / (ymax - ymin) + ymin)); 
	const double endscaled = rows * (1. - (((m * xmax + c) - ymin) / (ymax - ymin) + ymin));
	
	out = Image::zeros(rows, cols, CV_8UC3);
	
	for(const P2D& p : pts)
	{
		cv::Point2f pscaled;
		pscaled.x = cols * (p[0] - xmin) / (xmax - xmin);
		pscaled.y = rows * (1. - (p[1] - ymin) / (ymax - ymin)); 
			
		cv::circle(out, pscaled, 2, cv::Scalar(0, 255, 255));
	}
	
	cv::line(out, cv::Point(0, cscaled), cv::Point(cols, endscaled), cv::Scalar(255,0,0));
}

//FIXME: bug in the display 
void display_all_data(const std::vector<std::vector<P2D>>& data, const std::vector<LineCoefficients>& coefs, Image& out)
{
	const std::size_t rows = 500;
	const std::size_t cols = 800;
	
	const double xmax = 0.3;
	const double xmin = 0.0;
	const double ymin = -0.08;
	const double ymax = 0.02;
	
	out = Image::zeros(rows, cols, CV_8UC3);
	
	cv::Scalar colors_pts[] = {
		cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), 
		cv::Scalar(127, 0, 0), cv::Scalar(0, 127, 0), cv::Scalar(0, 0, 127)
	};
	int i=0;
	for(const auto& pts : data)
	{	
		for(const P2D& p : pts)
		{
			cv::Point2f pscaled;
			pscaled.x = cols * (p[0] - xmin) / (xmax - xmin) + 3.*i;
			pscaled.y = rows * (1. - (p[1] - ymin) / (ymax - ymin)); 
				
			cv::circle(out, pscaled, 1, colors_pts[i]);
		}
		i++;
	}
#if 1	
	cv::Scalar colors_lines[] = {
		cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), 
		cv::Scalar(127, 127, 0), cv::Scalar(0, 127, 127), cv::Scalar(127, 0, 127)
	};
	i=0;
	for(const auto & [m,c] : coefs)
	{		
		const double cscaled = rows * (1. - ((c - ymin) / (ymax - ymin) + ymin)); 
		const double endscaled = rows * (1. - (((m * xmax + c) - ymin) / (ymax - ymin) + ymin));
		DEBUG_VAR(endscaled);
	
		cv::line(out, cv::Point(0, cscaled), cv::Point(cols, endscaled), colors_lines[i++]);
	}
#endif
}
