#include <vector>

#include "types.h"

#include "geometry/mia.h"
#include "geometry/internals.h"

////////////////////////////////////////////////////////////////////////////////
void hist_data(const std::vector<double>& data, Image& dst, int binSize = 3, int height = 0, int nbbin = 250);
void hist_radii(const std::vector<MicroImage>& data);

////////////////////////////////////////////////////////////////////////////////
void display_data(const std::vector<P2D>& pts, const LineCoefficients& coefs, Image& out);
void display_all_data(const std::vector<std::vector<P2D>>& data, const std::vector<LineCoefficients>& coefs, Image& out);

////////////////////////////////////////////////////////////////////////////////
void compute_radii(const Image& img, const MIA& centers, std::vector<MicroImage>& data, std::size_t I=3u);
void compute_coefs(const std::vector<std::vector<P2D>>& data, std::vector<LineCoefficients>& coefs);

InternalParameters
preprocess(
	const std::vector<ImageWithInfo>& imgs, 
	const MIA& grid, 
	double pxl2metric = 0.0055,
	std::size_t I = 3u
);
