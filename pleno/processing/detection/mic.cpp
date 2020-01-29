#include "detection.h"

#include <opencv2/imgproc.hpp>

#include "io/printer.h"

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "processing/improcess.h"

static void imerode(const Image& raw, Image& preprocessed)
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
    Image eroded_image;
    erode(preprocessed, preprocessed, 3); // raytrix
    
    RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()++).name("Preprocessing::erroded"), preprocessed);
#endif
}

MICObservations detection_mic(const Image& raw)
{
////////////////////////////////////////Preprocessing///////////////////////////////////////////////
	Image preprocessed;
	imerode(raw /* in */, preprocessed /* out */);

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
        centers.emplace_back(
        	MICObservation{
 				-1, -1, // k,l
 				c.x, c.y //u,v
        	}
        );

    return centers;
}
