#pragma once

#include "types.h"
#include "geometry/mia.h"

////////////////////////////////////////////////////////////////////////////////
// DISPLAYS - Histograms
////////////////////////////////////////////////////////////////////////////////
void hist_data(const std::vector<double>& data, Image& dst, double min, double max, int binSize = 3, int height = 0, int nbbin = 250);
void hist_radii(const std::vector<MicroImage>& data);

////////////////////////////////////////////////////////////////////////////////
// DISPLAYS - data
////////////////////////////////////////////////////////////////////////////////
void display_data(const std::vector<P2D>& pts, const LineCoefficients& coefs, Image& out);
void display_all_data(const std::vector<std::vector<P2D>>& data, const std::vector<LineCoefficients>& coefs, Image& out);
