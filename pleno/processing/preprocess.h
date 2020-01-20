#include <vector>

#include "types.h"

#include "geometry/mia.h"
#include "geometry/internals.h"

////////////////////////////////////////////////////////////////////////////////
void hist_data(const std::vector<double>& data, Image& dst, int binSize = 3, int height = 0, int ref_value = -1);
void hist_radiance(const Image& img);
void hist_radii(const std::vector<MicroImage>& data);

////////////////////////////////////////////////////////////////////////////////
void display_data(const std::vector<P2D>& pts, const LineCoefficients& coefs, Image& out);
void display_all_data(const std::vector<std::vector<P2D>>& data, const std::vector<LineCoefficients>& coefs, Image& out);

////////////////////////////////////////////////////////////////////////////////
void compute_radii(const Image& img, const MIA& centers, std::vector<MicroImage>& data, bool trimmed = false, const std::vector<double>& hints = {});
void compute_coefs(const std::vector<std::vector<P2D>>& data, std::vector<LineCoefficients>& coefs);

InternalParameters
preprocess(
	const std::vector<ImageWithInfo>& imgs, 
	const MIA& grid, 
	double pxl2metric = 0.0055
);
