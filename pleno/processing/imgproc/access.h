#pragma once

#include "types.h"

////////////////////////////////////////////////////////////////////////////////
std::size_t size(const Image& i);
std::size_t width(const Image& i);
std::size_t height(const Image& i);

template<typename T> 
T get_pixel(const Image& i, size_t col, size_t row) { return i.at<T>(row, col); }

////////////////////////////////////////////////////////////////////////////////
Image extract_roi(const Image&img, double& X, double& Y, int roiw, int roih);
