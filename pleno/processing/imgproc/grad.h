#pragma once

#include "types.h"

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
