#include "interp.h"

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

