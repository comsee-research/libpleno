#pragma once

#include <type_traits>

#include "graphic/gui.h"

#include "graphic/viewer_3d.h"
#include "graphic/viewer_2d.h"

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/observation.h"
#include "geometry/reprojection.h"

#include "geometry/object/checkerboard.h"

inline void display(const CheckerBoard& checkboard)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("CheckerBoard"), 
		checkboard, 35.
	);
	Viewer::update(Viewer::Mode::m3D);
}

inline void display(const PlenopticCamera& model)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("CameraBody"), 
		model, tag::CameraBody{}
	);
	Viewer::update(Viewer::Mode::m3D);
	
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("TCM"), 
		model, tag::ThinLens{}, 35.
	);
	Viewer::update(Viewer::Mode::m3D);
	
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("MLA"), 
		model, tag::MLA{}, 5.
	);
	Viewer::update(Viewer::Mode::m3D);
	
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Sensor"), 
		model, tag::Sensor{}, 5.
	);
	Viewer::update(Viewer::Mode::m3D);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void display(const CalibrationPoses& poses)
{
	for(const auto& [p, f] : poses ) 
	{
		RENDER_DEBUG_3D(
			Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Pose ("+std::to_string(f)+")"), 
			p,
			35.0
		);
		Viewer::update(Viewer::Mode::m3D);
	}
}

inline void display(const Poses& poses)
{
	std::size_t i = 0;
	for(const auto& p : poses ) 
	{
		RENDER_DEBUG_3D(
			Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Pose (#"+std::to_string(i++)+")"), 
			p,
			35.0
		);
		Viewer::update(Viewer::Mode::m3D);
	}
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void display(int f /* frame */, const MICObservations& centers)
{
	v::Palette<int> palette;

	for(const auto& c : centers)
	{	
		RENDER_DEBUG_2D(
  			Viewer::context().layer(Viewer::layer())
  				.name("MI Centers ("+std::to_string(f)+")")
  				.pen_color(v::purple).pen_width(1),
  			P2D{c[0], c[1]}
		);	
	}
	Viewer::update();
}

inline void display(int f /* frame */, const CBObservations& corners)
{
	v::Palette<int> palette;

	for(const auto& c : corners)
	{	
		RENDER_DEBUG_2D(
  			Viewer::context().layer(Viewer::layer())
  				.name("Corners ("+std::to_string(f)+")")
  				.pen_color(v::purple).pen_width(1),
  			P2D{c[0], c[1]}
		);	
	}
	Viewer::update();
}

inline void display(int f /* frame */, const BAPObservations& obs)
{
	v::Palette<int> palette;
	
	for(const auto& o : obs)
	{	
		RENDER_DEBUG_2D(
  			Viewer::context().layer(Viewer::layer())
  				.name("BAP ("+std::to_string(f)+")")
  				.pen_color(palette[o.cluster]).pen_width(3),
  			Disk{o[0], o[1], std::fabs(o[2])}
		);	
	}
	Viewer::update();
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
template<typename Observations>
inline void display(int f /* frame */, const Observations& barycenters, tag::Barycenters)
{
	v::Palette<int> palette;
	
	for(const auto& b : barycenters)
	{
		RENDER_DEBUG_2D(
  			Viewer::context().layer(Viewer::layer())
  				.name("Barycenters ("+std::to_string(f)+")")
  				.pen_color(palette[b.cluster]).pen_width(3)
				.add_text(b[0], b[1] - 5, std::to_string(b.cluster)),
  			Disk{b[0], b[1], 50} //FIXME: 50 is arbitrary
		);	
	}
	Viewer::update();
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
template<typename Observations>
inline void display(int f /* frame */, const MICObservations& centers, const Observations& obs, const Observations& barycenters)
{
	display(f, centers);
	display(f, obs);
	display(f, barycenters, tag::Barycenters{});
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
template<typename Observations>
inline void display(
	const PlenopticCamera& mfpc,
	const CalibrationPoses& poses,
	const CheckerBoard & grid,
	const Observations& observations,
	const MICObservations& centers,
	const std::vector<Image>& pictures
)
{
	//Split observations according to frame
	std::unordered_map<int /* frame index */, Observations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
		
	auto model = mfpc;
	
	display(grid); display(poses);
	
	v::Palette<int> palette;

	const Viewer::Layer center_layer = 100;
	for(const auto& c : centers)
	{	
		RENDER_DEBUG_2D(
  			Viewer::context().layer(center_layer)
  				.name("Center")
  				.pen_color(v::purple).pen_width(1),
  			P2D{c[0], c[1]}
		);
		const auto p = reproject_miccenter(model, c);
		RENDER_DEBUG_2D(
  			Viewer::context().layer(center_layer+1)
  				.name("Reprojected Center")
  				.pen_color(v::cyan).pen_width(1),
  			p
		);		
	}
	Viewer::update();
	Viewer::update();

//For each frame 
	for(auto & [f, ob] : obs)
	{
		PRINT_DEBUG("Display information of frame f = " << f);
		
		for(const auto& [p,f] : poses) if(f == ob[0].frame) model.pose() = p;
		display(model);
#if 1		
		RENDER_DEBUG_2D(
			Viewer::context().layer(Viewer::layer()++).name("Frame f = "+std::to_string(f)), 
			pictures[f]
		);
#endif	
		for(const auto& o : ob)
		{				
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()++)
	  				.name("BAP ("+std::to_string(f)+")")
	  				.point_style(v::Pixel)
	  				.pen_color(palette[o.cluster]).pen_width(2),
	  			Disk{o[0], o[1], (std::is_same_v<Observations, BAPObservations>?o[2]:0.0)}
			);
			
			const P2D corner = reproject_corner(model, model.pose(), grid, o);
			const double radius = (std::is_same_v<Observations, BAPObservations>) ? reproject_radius(model, model.pose(), grid, o) : 0.0;

			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer()--)
	  				.name("Reprojected BAP ("+std::to_string(f)+")")
					.point_style(v::Cross)
	  				.pen_color(palette[o.cluster+1]).pen_width(2),
	  			Disk{corner, radius}
			);
		}

		Viewer::context().point_style(v::Pixel); //restore point style
		Viewer::update();
		Viewer::update();
		std::getchar();	
	}
}
