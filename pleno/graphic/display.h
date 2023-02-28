#pragma once

#include <type_traits>

#include "graphic/gui.h"

#include "graphic/viewer_3d.h"
#include "graphic/viewer_2d.h"

#include "io/choice.h"

#include "types.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/observation.h"
#include "geometry/reprojection.h"
#include "geometry/plane.h"
#include "geometry/ray.h"

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"
#include "geometry/depth/depthimage.h"

#include "geometry/object/checkerboard.h"
#include "geometry/object/constellation.h"

inline void display(const CheckerBoard& checkboard)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("CheckerBoard"), 
		checkboard, 35.
	);
	Viewer::update(Viewer::Mode::m3D);
}

inline void display(const Plate& plate)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Plate"), 
		plate
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
inline void display(const Image& white, const MIA& mia)
{
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("White image"), 
		white
	);
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name("Micro-Image Array")
			.pen_color(v::red).pen_width(2),
		mia
	);
	
	Viewer::update();
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

inline void display(const CalibrationPose& p)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Pose ("+std::to_string(p.frame)+")"), 
		p.pose,
		35.0
	);
	Viewer::update(Viewer::Mode::m3D);
}

inline void display(const Pose& p)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Pose (#-1)"), 
		p,
		35.0
	);
	Viewer::update(Viewer::Mode::m3D);
}

inline void display(const PointsConstellation& p, double scale = 5.)
{
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D)).name("Constellation"), 
		p, scale
	);
	Viewer::update(Viewer::Mode::m3D);
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
	const IndexedImages& pictures
)
{
	const bool usePictures = (pictures.size() > 0u);
	
	//Split observations according to frame
	std::unordered_map<int /* frame index */, Observations> obs;
	for(const auto& ob : observations)
		obs[ob.frame].push_back(ob);
		
	auto model = mfpc;
	
	display(grid); display(poses);
	
	v::Palette<int> palette;

	const Viewer::Layer layer = Viewer::layer();
	for(const auto& c : centers)
	{	
		RENDER_DEBUG_2D(
  			Viewer::context().layer(layer)
  				.name("Center")
  				.pen_color(v::purple).pen_width(1),
  			P2D{c[0], c[1]}
		);
		const auto p = reproject_miccenter(model, c);
		RENDER_DEBUG_2D(
  			Viewer::context().layer(layer+1)
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
		if (usePictures)
		{
			try { pictures.at(f); }
			catch (std::out_of_range&) { continue; }
		}
		PRINT_DEBUG("Display information of frame f = " << f);
		
		for(const auto& [p,f] : poses) if(f == ob[0].frame) model.pose() = p;
		display(model);
		
		if (usePictures)
		{
			RENDER_DEBUG_2D(
				Viewer::context().layer(Viewer::layer()).name("Frame f = "+std::to_string(f)), 
				pictures.at(f)
			);
		
			Viewer::update();
		}
		
		//get theorical reprojection
		Observations tbaps; int cluster = 0;
		for (const auto& pw : grid)
		{
			const P3D pc = to_coordinate_system_of(model.pose(), pw); // CAMERA
			
			Observations temp;
			mfpc.project(pc, temp);
			for (auto& o : temp) { o.frame = f; o.cluster = cluster; }
			
			tbaps.insert(tbaps.end(), std::make_move_iterator(temp.begin()), std::make_move_iterator(temp.end()));
			++cluster;
		}
		
		const Viewer::Layer layer = Viewer::layer();
		for (const auto& o : ob)
		{				
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(layer)
	  				.name("BAP ("+std::to_string(f)+")")
	  				.point_style(v::Round)
	  				.pen_color(palette[o.cluster]).pen_width(2),
	  			Disk{o[0], o[1], (std::is_same_v<Observations, BAPObservations>?o[2]:0.0)}
			);
			
			const P2D corner = reproject_corner(model, model.pose(), grid, o);
			const double radius = (mfpc.I() > 0u and std::is_same_v<Observations, BAPObservations>) ? reproject_radius(model, model.pose(), grid, o) : 0.0;

			RENDER_DEBUG_2D(
	  			Viewer::context().layer(layer+1)
	  				.name("Reprojected BAP ("+std::to_string(f)+")")
					.point_style(v::Cross)
	  				.pen_color(palette[o.cluster+1]).pen_width(2),
	  			Disk{corner, radius}
			);
		}
		
		Viewer::update();
		Viewer::update();
		
		for (const auto& o : tbaps)
		{
			RENDER_DEBUG_2D(
	  			Viewer::context().layer(Viewer::layer())
	  				.name("Theorical BAP ("+std::to_string(f)+")")
	  				.point_style(v::Cross)
	  				.pen_style(v::DashLine)
	  				.pen_color(palette[o.cluster]).pen_width(2),
	  			Disk{o[0], o[1], (std::is_same_v<Observations, BAPObservations>?o[2]:0.0)}
			);
		}

		Viewer::context().point_style(v::Pixel).pen_style(v::SolidLine); //restore point style
		Viewer::update();
		
		wait();
	}
}


//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
inline void display(const DepthMap& dm, const PlenopticCamera& pcm)
{
GUI(	
	std::string ss = (dm.is_virtual_depth()?"":"Metric ");
	
	const DepthMapImage dmi = DepthMapImage{dm, pcm};
	
	Image idm; cv::cvtColor(dmi.image, idm, cv::COLOR_BGR2RGB);
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()++)
			.name(ss+"Depth Map"),
		idm
  	);

  	// the legend
	constexpr int H = 50;
	const int W = idm.cols;
	
	cv::Mat legend;
	cv::resize(dmi.colormap, legend, cv::Size(W, H), 0, 0, cv::INTER_AREA);
	cv::imwrite("cm-legend.png", legend);
	cv::cvtColor(legend, legend, cv::COLOR_BGR2RGB);
	
	ss = (dm.is_virtual_depth()?"":" (mm)");
	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer())
			.name("Legend"+ss),
		legend,
		0, -2 * H
  	);
  	
  	const double nbsample = dm.is_virtual_depth()? std::ceil(dm.max_depth() - dm.min_depth()) : 15.;
  	const double stepv = dm.is_virtual_depth()? 1. : (dm.max_depth() - dm.min_depth()) / nbsample;
  	const double steppix = W / nbsample;
  	for (double v = dm.min_depth(), offsetpix = -5.; v <= dm.max_depth(); v += stepv, offsetpix += steppix)
  	{
  		std::ostringstream oss;
  		oss.precision((dm.is_virtual_depth()?2:4));
  		oss << v;
  		
  		Viewer::context().layer(Viewer::layer())
  			.font_size(50)
  			.pen_color(v::white)
  			.add_text(
  				offsetpix, - 2.5 * H,
  				oss.str() 			
  			);	  	
  	}
	Viewer::context().layer(Viewer::layer()++).update();  
	
	if (dm.is_coarse_map())
	{	
		constexpr std::size_t BORDER_MARGIN = 0;

		const std::size_t kmax = dm.width() - BORDER_MARGIN; 
		const std::size_t kmin = 0 + BORDER_MARGIN;
		const std::size_t lmax = dm.height() - BORDER_MARGIN; 
		const std::size_t lmin = 0 + BORDER_MARGIN;
	
		ss = (dm.is_virtual_depth()?"":"Metric ");  	
		for(std::size_t k = kmin; k < kmax; ++k)
		{
			for(std::size_t l = lmin; l < lmax; ++l)
			{
				const auto center = pcm.mia().nodeInWorld(k,l);	
				std::ostringstream oss;
		  		oss.precision((dm.is_virtual_depth()?3:4));
		  		oss << dm.depth(k,l)*(dm.is_virtual_depth()?10.:1.);		
				
				Viewer::context().layer(Viewer::layer())
	  				.font_size(5)
					.name(ss+"Depth Values")
		  			.pen_color(v::white)
		  			.add_text(
		  				center[0], center[1],
		  				oss.str()	  			
		  			);	
			}
		}
		
		Viewer::context().layer(Viewer::layer()++).update();  
	}
);
}

inline void display(int f /* frame */, const PointCloud& pc)
{	
GUI(
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D)
			.layer(Viewer::layer(Viewer::Mode::m3D))
			.name("PointCloud ("+std::to_string(f)+")"), 
		pc
	);
	Viewer::update(Viewer::Mode::m3D);
);
}

inline void display(int f /* frame */, const Plane& plane)
{	
GUI(
	RENDER_DEBUG_3D(
		Viewer::context(Viewer::Mode::m3D)
			.layer(Viewer::layer(Viewer::Mode::m3D))
			.name("Plane ("+std::to_string(f)+")"), 
		plane
	);
	Viewer::update(Viewer::Mode::m3D);
);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
template<typename Observations>
inline void display(
	const PlenopticCamera& mfpc,
	const CalibrationPose& pose,
	const PointsConstellation& constellation,
	const Observations& obs,
	const Image& picture
)
{	
GUI(
	auto model = mfpc;
	
	display(constellation); display(pose);
	
	v::Palette<int> palette;
	
	PRINT_DEBUG("Display information of frame");
		
	//model.pose() = pose.pose;
	display(model);

	RENDER_DEBUG_2D(
		Viewer::context().layer(Viewer::layer()).name("Frame"), 
		picture
	);
	
	Viewer::update();
	
	//get theorical reprojection
	Observations tbaps; int cluster = 0;
	for (const auto& pw : constellation)
	{
		const P3D pc = to_coordinate_system_of(pose.pose, pw); // CAMERA
		
		Observations temp;
		mfpc.project(pc, temp);
		for (auto& o : temp) { o.cluster = cluster; }
		
		tbaps.insert(tbaps.end(), std::make_move_iterator(temp.begin()), std::make_move_iterator(temp.end()));
		++cluster;
	}
	
//For each ob
	const Viewer::Layer layer = Viewer::layer();
	for (const auto& o : obs)
	{					
		RENDER_DEBUG_2D(
  			Viewer::context().layer(layer)
  				.name("BAP")
  				.point_style(v::Round)
  				.pen_color(palette[o.cluster]).pen_width(2),
  			Disk{o[0], o[1], (std::is_same_v<Observations, BAPObservations>?o[2]:0.0)}
		);
		
		const P3D pc = to_coordinate_system_of(pose.pose, constellation.get(o.cluster));
	
		P2D corner; model.project(pc, o.k, o.l, corner);
		double radius = 0.;
		if (mfpc.focused() and std::is_same_v<Observations, BAPObservations>) model.project(pc, o.k, o.l, radius);

		RENDER_DEBUG_2D(
  			Viewer::context().layer(layer+1)
  				.name("Reprojected BAP")
				.point_style(v::Cross)
  				.pen_color(palette[o.cluster+1]).pen_width(2),
  			Disk{corner, radius}
		);
	}
		
	Viewer::update();
	Viewer::update();
	
	for (const auto& o : tbaps)
	{
		RENDER_DEBUG_2D(
  			Viewer::context().layer(Viewer::layer())
  				.name("Theorical BAP")
  				.point_style(v::Cross)
  				.pen_style(v::DashLine)
  				.pen_color(palette[o.cluster]).pen_width(2),
  			Disk{o[0], o[1], (std::is_same_v<Observations, BAPObservations>?o[2]:0.0)}
		);
	}

	Viewer::context().point_style(v::Pixel).pen_style(v::SolidLine); //restore point style
	Viewer::update();
);
}
