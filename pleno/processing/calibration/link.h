#pragma once

#include "types.h"
#include "unused.h"

#include <unordered_map>
#include <vector>

#include "processing/algorithms/neighbour_search.h"
#include "processing/algorithms/p3p.h"

#include "geometry/object/checkerboard.h"
#include "geometry/object/constellation.h" //PointsConstellation
#include "geometry/camera/camera.h"

#include "geometry/pose.h"
#include "geometry/mia.h"
#include "geometry/observation.h"

#include "processing/tools/matrix.h"


//******************************************************************************
template<typename Observations> Observations compute_barycenters(const Observations& observations);
template<typename Observations> void get_4_corners(const Observations& observations, Observations& corners); /* tl - tr - br - bl */

//******************************************************************************
template<typename Observations, typename CameraModel>
void link_cluster_to_node_index(
    Observations& observations, /* in/out */
    const Observations& barycenters,
	const CameraModel& monocular, 
	const CheckerBoard& grid,
    const Pose& pose
);
void link_center_to_node_index(MICObservations& centers, const MIA& grid);

//******************************************************************************
template<typename Observations>
void link_cluster_to_point_constellation_index(
	Observations& observations,
	const PlenopticCamera& pcm, 
	const PointsConstellation& constellation,	
	const Image& gray
);

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observations>
Observations compute_barycenters(const Observations& observations) 
{
	using Observation = typename Observations::value_type;
	
	//Split observations according to cluster
	std::unordered_map<int /* cluster index */, Observations> obs;
	for(const auto& ob : observations)
		obs[ob.cluster].push_back(ob);
		
	Observations barycenters;
	barycenters.reserve(obs.size());
	
	struct Accumulator {
		double u, v;
		double n = 0;
	};
		
	for(const auto & [c, ob] : obs)
	{		
		Accumulator acc = std::accumulate(
			ob.begin(),
			ob.end(), 
			Accumulator{0.,0.},
			[](Accumulator acc, const Observation& current) {
				return Accumulator{acc.u + current[0], acc.v + current[1], acc.n+1};
			}
		);
		
		Observation temp; 
			temp[0] = acc.u / acc.n; 
			temp[1] = acc.v / acc.n;
			temp.cluster = c;
			temp.frame = ob[0].frame;
		
		barycenters.emplace_back(
			std::move(temp) //FIXME: check
		);
	}
	
	return barycenters;
}

template<typename Observations>
void get_4_corners(const Observations& observations, Observations& corners) /* tl - tr - br - bl */
{
    corners.clear();
    corners.resize(4); /* tl - tr - br - bl */
    
    Observations obs{observations.begin(), observations.end()};
	auto accessor = [](const auto& c) { return P2D{c[0], c[1]}; };
	
	const auto& [maxx, maxy] = [&](const Observations& obs) -> std::pair<double, double> {
		double maxx=-1e12, maxy=-1e12;
		for(const auto&ob : obs) {
			const double x = ob[0];
			const double y = ob[1];
			if(x > maxx) maxx = x;
			if(y > maxy) maxy = y;
		}
		return {std::ceil(maxx), std::ceil(maxy)};
	}(obs); 

	//---bottom-right
	P2D br = FNS::find(obs, P2D{0.0, 0.0}, accessor);
	corners[Corner::BR][0] = br[0]; corners[Corner::BR][1] = br[1];
	
	//---top-left
	//P2D tl = FNS::find(obs, P2D{maxx, maxy}, accessor);
	P2D tl = FNS::find(obs, br, accessor); UNUSED(maxx); UNUSED(maxy);
	corners[Corner::TL][0] = tl[0]; corners[Corner::TL][1] = tl[1];
	
	//---top-right    
    P2D tr = FNS::find(obs, P2D{tl[0], br[1]}, accessor);
	corners[Corner::TR][0] = tr[0]; corners[Corner::TR][1] = tr[1];
	
	//---bottom-left    
    P2D bl = FNS::find(obs, P2D{br[0], tl[1]}, accessor);
	corners[Corner::BL][0] = bl[0]; corners[Corner::BL][1] = bl[1];
				
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observations, typename CameraModel>
void link_cluster_to_node_index(
    Observations& observations, /* in/out */
    const Observations& barycenters,
	const CameraModel& monocular, 
	const CheckerBoard& grid,
    const Pose& pose,
    bool verbose = true
)
{
	std::unordered_map<int /* old id */, int /* new id */> id_mapping;
		
	//Find mean dist inter-reprojected node
	P2D p00, p01, p10;
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(0,0)), p00);
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(grid.width()-1, 0)), p01);
	monocular.project(to_coordinate_system_of(pose, grid.nodeInWorld(0, grid.height()-1)), p10);
	const double interdist = (  (p00 - p01).norm() / static_cast<double>(grid.width()-1)
							 + 	(p00 - p10).norm() / static_cast<double>(grid.height()-1)
							 ) / 2. ;
	constexpr double tolerance = 0.6;
	
	//For each checkerboard node
	for(std::size_t k = 0; k < grid.width(); ++k) //iterate through columns //x-axis
    {
    	for(std::size_t l = 0; l < grid.height(); ++l) //iterate through lines //y-axis
		{
			const int id = l * grid.width() + k; //node id
			
			//Project node in image
			const P3D p3d_cam = to_coordinate_system_of(pose, grid.nodeInWorld(k,l));	
			P2D projection; // a projected checkerboard node in IMAGE UV
        	bool projected = monocular.project(p3d_cam, projection);
        	       	
        	if ((not projected)) 
        	{ 
        		if(verbose) PRINT_ERR("CheckerBoard Node ("<<k<<", "<<l<<") not reprojected in image : "<<projection); 
        		continue; 
        	}	
        	
        	//Find nearest cluster
        	int cluster = -1;
        	double dist = 1e20;
        	for(const auto &ob : barycenters)
        	{
        		const double new_dist = std::hypot(ob[0] - projection[0], ob[1] - projection[1]);
        		if( new_dist < dist and new_dist < interdist * tolerance) 
        		{
        			dist = new_dist;
        			cluster = ob.cluster;
        		}
        	}
        	
        	if(verbose) 
        	{
				GUI(
					RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer())
						.point_style(v::Cross).pen_color(v::cyan).pen_width(5)
						.add_text(projection[0], projection[1] + 5, "("+std::to_string(k)+", "+std::to_string(l)+")")
						.name("Projected nodes"),
						projection
					);
				);
			}
			
			if(cluster != -1 and id_mapping.count(cluster) == 0) 
				id_mapping[cluster] = id;
		}
	}
	if(verbose) 
    {
		GUI(
			Viewer::context().point_style(v::Pixel); //restore point style
			Viewer::update();
		);
	}
	//Assign new cluster id
	for (auto& o : observations)
	{
		if(id_mapping.count(o.cluster) > 0) 
		{
			o.cluster = id_mapping[o.cluster];
			o.isValid = true;	
		}
		else
		{
			o.cluster = -1;
			o.isValid = false;		
		}
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline void link_center_to_node_index(
	MICObservations& centers, //in/out
	const MIA& grid
)
{
	constexpr double tolerance = 0.7;
	const double interdist = grid.pitch()[0];
	const int size = static_cast<int>(grid.nodeNbr());
	
	//Invalidate all observations
	std::for_each(
    	centers.begin(), centers.end(),
    	[](MICObservation& center) { center.isValid = false; }
    );
	
	// for each grid node
#pragma omp parallel for
    for (int index = 0; index < size ; ++index)
    {
        const P2D node = grid.nodeInWorld(index);
        
        //Find nearest center
    	int nearest_id = -1, center_id = 0;
    	double dist = 1e20;
    	for(const auto &c : centers)
    	{
    		const double new_dist = (P2D{c[0],c[1]} - node).norm(); 
    		if( new_dist < dist and new_dist < interdist * tolerance) 
    		{
    			dist = new_dist;
    			nearest_id = center_id;
    		}
    		
    		++center_id;
    	}
        
        if(nearest_id != -1) //if a center is found
        {
        	const auto kl = index_to_colRow(grid.width(), index);
        	centers[nearest_id].k = kl[0]; centers[nearest_id].l = kl[1]; 
        	centers[nearest_id].isValid = true;
        }
   }
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observations>
void link_cluster_to_point_constellation_index(
	Observations& observations,
	const PlenopticCamera& pcm, 
	const PointsConstellation& constellation,	
	const Image& gray
)
{
	PRINT_WARN("Link requires manual intervention.");
	
	std::unordered_map<int /* old id */, int /* new id */> id_mapping;
	
FORCE_GUI(true);
	const double ratio = double(gray.rows) / double(gray.cols);
	const int base_size = 800;
	
	GUI(
		RENDER_DEBUG_2D(
			Viewer::context().size(base_size,base_size*ratio).layer(Viewer::layer()).name("Scene"), 
			gray
		);
    );
    Viewer::update();	
	
	//compute barycenters
	const Observations barycenters = compute_barycenters(observations); DEBUG_VAR(barycenters.size());
	display(-1, barycenters, tag::Barycenters{});
	
	//for each point in constellation
	for (std::size_t i = 0; i < constellation.size(); ++i)
	{
		volatile bool finished = false;
		
		P2D point; 
		
		PRINT_INFO("Click on cluster corresponding to point (" << i << ") in constellation");	
		Viewer::context().on_click([&](float x, float y){
			if (x > 0. and x < pcm.sensor().width() and y > 0. and y < pcm.sensor().height())
			{
				PRINT_DEBUG("Click at position ("<< x << ", "<< y << ")");
				GUI(
					P2D c = P2D{x,y};
		  			Viewer::context().layer(Viewer::layer())
		  				.name("Selected cluster")
		  				.pen_color(v::purple).pen_width(5)
		  				.add_circle(c[0], c[1], 25.)
		  				.update();
				);
			
				point.x() = x;
				point.y() = y;			
				finished = true;
			}
			else
			{
				PRINT_ERR("Click out of bound, unvalidated. Try again.");
				return;
			}
		});
		
		//wait for click	
		while(not finished);
		Viewer::context().on_click([&](float, float){});
		
		//find nearest cluster
		int minc = -1; double mindist = 1e5; int j = 0;
		for (const auto& center : barycenters)
		{
			const double dist = (P2D{center.u, center.v} - point).norm(); 
			if (dist < mindist)
			{
				minc = j;
				mindist = dist;
			}
			++j;
		}
		
		id_mapping[minc] = i;							
	}
	
	//assign new cluster id
	for (auto& o : observations)
	{
		if(id_mapping.count(o.cluster) > 0) 
		{
			o.cluster = id_mapping[o.cluster];
			o.isValid = true;	
		}
		else
		{
			o.cluster = -1;
			o.isValid = false;		
		}
	}

FORCE_GUI(false);
}

