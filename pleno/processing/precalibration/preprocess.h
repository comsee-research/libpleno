#include <vector>

#include "types.h"

#include "geometry/mia.h"
#include "geometry/internals.h"
#include "geometry/camera/plenoptic.h"

////////////////////////////////////////////////////////////////////////////////
void compute_radii(const Image& img, const MIA& centers, std::vector<MicroImage>& data, std::size_t I=3u);
void compute_coefs(const std::vector<std::vector<P2D>>& data, std::vector<LineCoefficients>& coefs);

InternalParameters
preprocess(
	const std::vector<ImageWithInfo>& imgs, 
	const MIA& grid, 
	double pxl2metric = 0.0055,
	std::size_t I = 3u,
	int mode = PlenopticCamera::Mode::Galilean,
	double fmatchingnumber = 4.
);
