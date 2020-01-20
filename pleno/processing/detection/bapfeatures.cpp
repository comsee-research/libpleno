#include "detection.h"

#include <algorithm>
#include <unordered_map>

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"

BAPObservations
detection_bapfeatures(const Image& raw, const MIA& mia, const InternalParameters& params)
{
	PRINT_INFO("=== Detecting corners in image");
	CBObservations cbos = detection_corners(raw /* img */, mia, params);
	
	PRINT_INFO("=== Computing BAP Features in image");
	BAPObservations bapobs = compute_bapfeatures(cbos, mia, params);
	
	GUI(
		v::Palette<int> palette;
		for(const auto& ob: bapobs)
		{
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer())
	  				.name("BAP")
	  				.pen_color(palette[ob.cluster]).pen_width(2),
	  			Disk{P2D{ob.u, ob.v}, ob.rho}
			);			
		}
		Viewer::update();
	);
	
	bapobs.shrink_to_fit();
	return bapobs;
}
