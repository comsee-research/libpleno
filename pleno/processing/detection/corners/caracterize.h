#pragma once

#include "types.h"

#include "model/microimage.h"

/*
 * @orientation_histogram compute an histogram based on orientation and magnitude image gradients
 */
std::vector<double>
hist_orientation(
	const Image& orientation, 
	const Image& magnitude,
    const double stepsize,
	const Image& mask    
);

/*
 * @orientation_histogram compute an histogram based on orientation and magnitude image gradients
 */
std::vector<double>
hist_orientation_modulo_180(
	const Image& orientation, 
	const Image& magnitude,
    const double stepsize,
	const Image& mask    
);

std::vector<double>
compute_polar_histogram(
    const Image& img,
    const Image& mask,
    const double scale = 1.0
);

/**
 * @Brief 	Filter peaks according to the max angle distance separating them. 
 *			Pack together too closed peaks, and mean the magnitude.
 **/
void filter_peaks(Peaks& peaks, const double maxdist);

MicroImageModel compute_model(const Image& mi, const Image& mask);

MicroImageType caracterize(const Image& mi, const Image& mask);
