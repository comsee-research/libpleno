#include "improcess.h"

std::size_t size(const Image& i) 	{ return i.cols * i.rows; }
std::size_t width(const Image& i)	{ return i.cols; }
std::size_t height(const Image& i) 	{ return i.rows; }

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
Image extract_roi(const Image&img, float& X, float& Y, int roiw, int roih)
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void trim(Image& img, float r, float tolerance)
{
	const int width 	= img.cols;
	const int height 	= img.rows;
	const float centerx = width/2.;
	const float centery = height/2.;
	
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

void devignetting(const Image& raw, const Image& white, Image& unvignetted)
{    
    unvignetted = raw.clone();

    for (int row = 0; row < unvignetted.rows; ++row)
    {
        for (int col = 0; col < unvignetted.cols; ++col)
        {            
            auto & w = white.at<cv::Vec3b>(row, col);              
            auto & u = unvignetted.at<cv::Vec3b>(row, col);
            u[0] = static_cast<uchar>( std::min(255.f, u[0] * 255.f / (w[0] + 1e-3f)) );
            u[1] = static_cast<uchar>( std::min(255.f, u[1] * 255.f / (w[1] + 1e-3f)) );
            u[2] = static_cast<uchar>( std::min(255.f, u[2] * 255.f / (w[2] + 1e-3f)) );
        }
    }
}

void contrast_strech(const Image& input, Image& output, const int threshold)
{
    for (int row = 0; row < output.rows; ++row)
        for (int col = 0; col < output.cols; ++col)
        {
            unsigned char intensity = input.at<unsigned char>(row, col);
            if (intensity < threshold)
                intensity = 0;

            output.at<unsigned char>(row, col) = intensity;
        }
}

void erode(const Image& input, Image& output, const int crossSize)
{
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
                                            cv::Size(2 * crossSize + 1, 2 * crossSize + 1),
                                            cv::Point(crossSize, crossSize) );

    cv::erode(input, output, element);
}

void dilate(const Image& input, Image& output, const int crossSize)
{
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
                                            cv::Size(2 * crossSize + 1, 2 * crossSize + 1),
                                            cv::Point(crossSize, crossSize) );

    cv::dilate(input, output, element);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<cv::Point>> detect_shapes(Image& ioSource)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(ioSource, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE);

    cv::RNG rng (12345);
    cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(ioSource, contours, -1, color, 0, 8, hierarchy, INT_MAX);

    return contours;
}

void fit_polygons(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Point2f>& center)
{
    std::vector<std::vector<cv::Point>> contours_polygon(contours.size());
    std::vector<cv::Rect> boundRect(contours.size());
    std::vector<float> radius(contours.size());

    for (size_t c = 0; c < contours.size(); ++c )
    {
        cv::approxPolyDP(cv::Mat(contours[c]), contours_polygon[c], 3, true);
        boundRect[c] = boundingRect(cv::Mat(contours_polygon[c]));
        cv::minEnclosingCircle(cv::Mat(contours_polygon[c]), center[c], radius[c]);
    }
}

void hist_radiance(const Image& img, Image& histogram)
{
	/// Establish the number of bins
	int histSize = 256;

	/// Set the ranges ( for B,G,R) )
	float range[] = { 0, 256 } ;
	const float* histRange = { range };

	bool uniform = true; 
	bool accumulate = false;
	
	Image hist;
	calcHist( &img, 1, 0, Image(), hist, 1, &histSize, &histRange, uniform, accumulate);
	
	// Draw the histograms for R, G and B
	int hist_w = 512; int hist_h = 400;
	int bin_w = cvRound( (double) hist_w/histSize );

	Image histImage(hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0));
	
	/// Normalize the result to [ 0, histImage.rows ]
	cv::normalize(hist, hist, 0, histImage.rows, cv::NORM_MINMAX, -1, Image());
	
	for( int i = 1; i < histSize; i++ )
	{
	  	cv::line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
		               cv::Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
		               cv::Scalar( 255, 0, 0), 2, 8, 0);
	}
	
    histogram = histImage; 
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/*
 * Apply transformation
 * TODO: add a last (col,row) in order to remove the black borders
 */
bool warp(const Transformation& t, const Image& input, Image& output)
{
    P2D pixel {0, 0};
    P3D new_p3d;

    const Image empty_mask(input.rows, input.cols, CV_64FC1, 1.0);
    GrayInterpolator interpolator{empty_mask};

    output = Image(input.rows, input.cols, CV_64FC1, -1.0);

    //starting interpolation
    for (int row = 0; row < input.rows; ++row)
    {
        for (int col = 0; col < input.cols; ++col)
        {
            pixel = {col, row};

            // transformation from scratch
            new_p3d = t() * pixel.homogeneous();

            // Normalisation: Put the transformed point on a plane
            pixel = new_p3d.head(2) / new_p3d.z();

            output.at<double>(row, col) = interpolator(input, pixel);
        }
    }

    return true;
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// return true is the pixel is valid
bool GrayInterpolator::is_valid(const P2D& pixel) const
{
  	// check if the pixel is in the image
  	if ( 	(pixel[0] >= 0) and (pixel[0] < mask.cols - 1)
		and (pixel[1] >= 0) and (pixel[1] < mask.rows - 1)
	)
	{
		return mask.at<double>(pixel[1], pixel[0]) != 0.0;
	}
	else 
	{
		return false;
	}
}

double GrayInterpolator::intensity(const cv::Mat& image, const P2D& pixel) const
{
    return image.at<double>(pixel[1], pixel[0]);
}

// Computing nearest rounded values neighbors of a point
P2DS GrayInterpolator::neighbors(const P2D& pixel) const
{
    P2DS neighbors;

    P2DS four_neighbors = {
        { int(std::floor(pixel[0])), int(std::floor(pixel[1])) }, // (0, 0)
        {  int(std::ceil(pixel[0])), int(std::floor(pixel[1])) }, // (1, 0)
        {  int(std::ceil(pixel[0])),  int(std::ceil(pixel[1])) }, // (1, 1)
        { int(std::floor(pixel[0])),  int(std::ceil(pixel[1])) }  // (0, 1)
    };

    // filtering neighbors with mask
    for (auto& p : four_neighbors)
    {
        if (is_valid(p))
        {
            neighbors.push_back(p);
        }
    }

    return neighbors;
}

double GrayInterpolator::operator()(const cv::Mat& image, const P2D& pixel) const
{
    // bilinear interpolation (4 neighbors)
    // f(x,y) \approx f(0,0) (1-x) (1-y)
                 // + f(1,0)     x (1-y) 
                 // + f(0,1) (1-x)     y 
                 // + f(1,1)     x     y

    const P2DS n = neighbors(pixel);

    // on  ramene les valeurs entre 0 et 1
    double x = pixel[0] - std::floor(pixel[0]);
    double y = pixel[1] - std::floor(pixel[1]);
    double vx = 1.0 - x;
    double vy = 1.0 - y;

    if (n.size() == 4)
    {
        return   vx * vy * intensity(image, n[0])
               +  x * vy * intensity(image, n[1])
               + vx *  y * intensity(image, n[3])
               +  x *  y * intensity(image, n[2]);
    }
    else if (n.size() == 3)
    {
        return   (1 - x - y) * intensity(image, n[0])
               +           x * intensity(image, n[1])
               +           y * intensity(image, n[2]);
    }
    else if (n.size() == 2)
    {
        // linerar interpolation
        double d = std::sqrt(x * x + y * y);

        return (1.0 - d) * intensity(image, n[0])
               +       d * intensity(image, n[1]);
    }
    else if (n.size() == 1)
    {
        return intensity(image, n[0]);
    }
    else
    {
        return -1.0;
    }

    return -1.0;
}


