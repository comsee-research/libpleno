#pragma once

#include "types.h"
#include "transformation.h"

////////////////////////////////////////////////////////////////////////////////
std::size_t size(const Image& i);
std::size_t width(const Image& i);
std::size_t height(const Image& i);

template<typename T> 
T get_pixel(const Image& i, size_t col, size_t row) { return i.at<T>(row, col); }

////////////////////////////////////////////////////////////////////////////////
Image extract_roi(const Image&img, float& X, float& Y, int roiw, int roih);

////////////////////////////////////////////////////////////////////////////////
void trim(Image& img, float r, float tolerance=0.);

void contrast_strech(const Image& input, Image& output, const int threshold);

void erode(const Image& input, Image& output, const int crossSize);
void dilate(const Image& input, Image& output, const int crossSize);

void devignetting(const Image& raw, const Image& white, Image& unvignetted);

////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<cv::Point>> detect_shapes(Image& ioSource);
void fit_polygons(std::vector<std::vector<cv::Point>>& shape, std::vector<cv::Point2f>& center);

void hist_radiance(const Image& img, Image& histogram);
////////////////////////////////////////////////////////////////////////////////
// compute the x_gradient of an image
template <typename T>
void x_gradient(const cv::Mat& input, cv::Mat& output)
{
    output = cv::Mat_<T>(input.rows, input.cols);

    for (int row = 0; row < output.rows; ++row)
    {
        // go through the first column
        output.at<T>(row, 0) = (input.at<T>(row, 1) - input.at<T>(row, 0));

        // go through all the image exept at the end
        for (int col = 1; col < output.cols - 1; ++col)
            output.at<T>(row, col) = (input.at<T>(row, col + 1) - input.at<T>(row, col - 1)) / 2.0;

        // go through the last column
        output.at<T>(row, output.cols - 1) = (input.at<T>(row, output.cols - 1)
                                            - input.at<T>(row, output.cols - 2));
    }
}

// compute the y_gradient of an image
template <typename T>
void y_gradient(const cv::Mat& input, cv::Mat& output)
{
    output = cv::Mat_<T>(input.rows, input.cols);

    for (int col = 0; col < output.cols; ++col)
    {
        // go through the first row
        output.at<T>(0, col) = (input.at<T>(1, col) - input.at<T>(0, col));

        // go through all the image exept at the end
        for (int row = 1; row < input.rows - 1; ++row)
            output.at<T>(row, col) = (input.at<T>(row + 1, col) - input.at<T>(row - 1, col)) / 2.0;

        // go through the last row
        output.at<T>(input.rows - 1, col) = (input.at<T>(input.rows - 1, col)
                                           - input.at<T>(input.rows - 2, col));
    }
}

////////////////////////////////////////////////////////////////////////////////
/*
 * Apply transformation
 * TODO: add a last (col,row) in order to remove the black borders
 */
bool warp(const Transformation& t, const Image& input, Image& output);

////////////////////////////////////////////////////////////////////////////////
struct GrayInterpolator {
    const Image& mask;

    bool is_valid(const P2D& pixel) const;

    double intensity(const Image& image, const P2D& pixel) const;

    // Computing nearest rounded values neighbors of a point
    P2DS neighbors(const P2D& pixel) const;

    double operator()(const Image& image, const P2D& pixel) const;
};
