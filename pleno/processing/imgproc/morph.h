#pragma once

#include "types.h"

////////////////////////////////////////////////////////////////////////////////
void erode(const Image& input, Image& output, int crossSize);
void dilate(const Image& input, Image& output, int crossSize);

////////////////////////////////////////////////////////////////////////////////
std::vector<std::vector<cv::Point>> detect_shapes(Image& ioSource);
void fit_polygons(std::vector<std::vector<cv::Point>>& shape, std::vector<cv::Point2f>& center);
