#pragma once

#include "graphic/gui.h"

#include "graphic/viewer_3d.h"
#include "graphic/viewer_2d.h"

#include "types.h"

#include "geometry/camera/mfpc.h"
#include "geometry/observation.h"

#include "geometry/object/checkerboard.h"

inline void display(const CheckerBoard& checkboard)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("CheckerBoard"), 
		checkboard, 35.
	);
	Viewer::update(Viewer::Mode::m3D);
}

inline void display(const MultiFocusPlenopticCamera& model)
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
template<typename Observations_t>
inline void display(int f /* frame */, const Observations_t& barycenters, tag::Barycenters)
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
template<typename Observations_t>
inline void display(int f /* frame */, const MICObservations& centers, const Observations_t& obs, const Observations_t& barycenters)
{
	display(f, centers);
	display(f, obs);
	display(f, barycenters, tag::Barycenters{});
}

