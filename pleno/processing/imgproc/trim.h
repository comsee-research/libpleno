#pragma once

#include "types.h"

////////////////////////////////////////////////////////////////////////////////
void trim(Image& img, double r, double tolerance = 0.);
void trim_binarize(Image& img, double radius, double tolerance = 0.);
void trim_float(Image& img, double r, double tolerance = 0.);
void trim_float_binarize(Image& img, double radius, double tolerance = 0.);
void trim_double(Image& img, double r, double tolerance = 0.);
void trim_double_binarize(Image& img, double radius, double tolerance = 0.);

