#include "detection.h"

#include <opencv2/imgproc.hpp>

#include "io/printer.h"

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "processing/imgproc/improcess.h"

static void contrast_strech(const Image& input, Image& output, int threshold)
{
    for (int row = 0; row < output.rows; ++row)
    {
        for (int col = 0; col < output.cols; ++col)
        {
            auto intensity = input.at<uchar>(row, col);
            if (intensity < threshold) intensity = 0;

            output.at<uchar>(row, col) = intensity;
        }
    }
}

static void impreprocess(const Image& raw, Image& preprocessed, std::size_t I)
{
/////////////////////////////////////Grayscale conversion///////////////////////////////////////////
    PRINT_INFO("Grayscale conversion");
    
    preprocessed = Image::zeros(raw.rows, raw.cols, CV_8UC1);
    cv::cvtColor(raw, preprocessed, cv::COLOR_BGR2GRAY);
    
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("Preprocessing::gray"), preprocessed);
    
#if 0
////////////////////////////////////////Equalization/////////////////////////////////////////////////
    PRINT_INFO("Equalization");
   
  	cv::equalizeHist( preprocessed, preprocessed);
  	
   	RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("Preprocessing::equalized"), preprocessed);
#endif	
 
////////////////////////////////////////Contrast stretching/////////////////////////////////////////
    PRINT_INFO("Contrast stretching");
    
    const cv::Scalar mean =	cv::mean(preprocessed);
    
    PRINT_DEBUG("mean: " << mean);
    contrast_strech(preprocessed, preprocessed, int(mean[0]));
   	RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("Preprocessing::contrasted"), preprocessed);
    
#if 1
//////////////////////////////////////////Eroding image/////////////////////////////////////////////
    PRINT_INFO("Eroding image");
    //Image eroded_image;
    erode(preprocessed, preprocessed, (I>0) ? 3 /* raytrix */ : 1 /* Lytro */);
    
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("Preprocessing::erroded"), preprocessed);
#endif
}

MICObservations detection_mic(const Image& raw, std::size_t I)
{
	constexpr std::size_t margin = 2ul; //pixel 

////////////////////////////////////////Preprocessing///////////////////////////////////////////////
	Image preprocessed;
	impreprocess(raw /* in */, preprocessed /* out */, I);

////////////////////////////////////////Detecting contours//////////////////////////////////////////
    PRINT_INFO("Detecting contours");
    std::vector<std::vector<cv::Point>> contours = detect_shapes(preprocessed);
    
/////////////////////////////Method Fitting polygon & Estimate centers///////////////////////////
    PRINT_INFO("Fitting polygon & Estimate centers");
    std::vector<cv::Point2f> unsorted_centers(contours.size()); //Lenslet centers
    fit_polygons(contours, unsorted_centers);

    PRINT_DEBUG("centers.size(): " << unsorted_centers.size());

    MICObservations centers;
    centers.reserve(unsorted_centers.size());
    
    for (const auto& c : unsorted_centers)
    {
    	if (c.x < margin or c.y < margin or c.x >= raw.cols-margin or c.y >= raw.rows-margin) continue;
    	
        centers.emplace_back(
        	MICObservation{
 				-1, -1, // k,l
 				c.x, c.y //u,v
        	}
        );
	}
    return centers;
}
