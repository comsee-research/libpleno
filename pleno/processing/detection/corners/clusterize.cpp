#include "clusterize.h"

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"

#include "processing/imgproc/improcess.h"
#include "processing/algorithms/dbscan.h"

CBObservations clusterize(const CBObservations& obs, double eps, std::size_t min_pts, bool filter)
{
	CBObservations filtered;
	filtered.reserve(obs.size());

//Clusterize
	std::vector<NNS::Index> idx(obs.size());
	std::iota(idx.begin(), idx.end(), NNS::Index(0));
	
	PRINT_DEBUG("Scanning...");
	DBSCAN<CheckerBoardObservation> dbscan(obs, eps, min_pts);
	
	dbscan.clusterize();
	PRINT_DEBUG("Scanned!");
	auto& clusters = dbscan.clusters;
	DEBUG_VAR(clusters.size());
	
	if(not clusters.empty())
	{
		GUI(
			v::Palette<std::size_t> palette;
			for (std::size_t i = 0; i < clusters.size(); ++i)
			{
				for(auto& id : clusters[i])
				{
					RENDER_DEBUG_2D(
			  			Viewer::context().layer(Viewer::layer())
			  				.name("clusters")
			  				.pen_color(palette[i]).pen_width(8),
			  			P2D{obs[id][0], obs[id][1]}
					);	
				}
			}		
			Viewer::update();
		);
		
	//Remove non-homogeneous clusters
		if(filter)
		{
			constexpr std::size_t magic_number = 6u;
			constexpr double tolerance = 0.5;
			constexpr double tolerance_sup = 0.8;
			
			auto medsize_f = [](std::vector<std::size_t> sizes) -> std::size_t {
				#if 0 //MEDIAN ESTIMATOR			
					std::nth_element(sizes.begin(), sizes.begin() + sizes.size() / 2, sizes.end());
					return sizes[sizes.size() / 2];
				#else //MEAN ESTIMATOR
					return std::accumulate(sizes.begin(), sizes.end(), 0.) / (sizes.size() + 1e-9);
				#endif
			};
						
			auto sizes_f = [](const auto& clusters) -> std::vector<std::size_t> {
				std::vector<std::size_t> sizes;
				std::transform(
					clusters.begin(), clusters.end(),
					std::back_inserter(sizes),
					[](const auto& c) -> std::size_t { return c.size(); }		
				);
				return sizes;
			};
			
			auto barycenter = [&obs](const auto& cluster) -> P2D {
				struct Accumulator { double u, v; double n = 0; };
				Accumulator acc = std::accumulate(
					cluster.begin(), cluster.end(), Accumulator{0.,0.},
					[&obs](Accumulator acc, const auto&id) {
						return Accumulator{acc.u + obs[id][0], acc.v + obs[id][1], acc.n+1};
					}
				);
				return P2D{acc.u / acc.n, acc.v / acc.n};
			};
			
			auto validate_remove = [&barycenter](const auto& cluster) -> bool {
				//Compute barycenter
				P2D p = barycenter(cluster);
				PRINT_WARN("Remove cluster at ("<<p[0]<<", "<<p[1]<<")? [y/n]");
				//Display barycenter
				RENDER_DEBUG_2D(
		  			Viewer::context().layer(Viewer::layer())
		  				.name("Cluster to remove?")
		  				.pen_color(v::red).pen_width(3),
		  			Disk{p, 50} //FIXME: 50 is arbitrary
				);	
				//Get choice
				bool ret = false;
				char c;
				std::cin >> c;
				if(c == 'y') { ret = true; }	
				std::cin.clear();
				while (std::cin.get() != '\n');	
				//Clean GUI
				GUI(
					Viewer::context().layer(Viewer::layer()).clear().update();
				);
				return ret;
			};
			
			//Remove too small clusters
			clusters.erase(
				std::remove_if(clusters.begin(), clusters.end(), 
					[&tolerance, &magic_number, medsize= medsize_f(sizes_f(clusters)), &validate_remove](const auto& c){
						bool ret = false;
						if((medsize > magic_number) and	(c.size() < medsize * (1. - tolerance)))	
							ret = validate_remove(c);				
						return ret;
					}
				),
				clusters.end()
			);
			clusters.shrink_to_fit();
						
			//Remove too big clusters
			clusters.erase(
				std::remove_if(clusters.begin(), clusters.end(), 
					[&tolerance_sup, &magic_number, medsize = medsize_f(sizes_f(clusters)), &validate_remove](const auto& c){
						bool ret = false;
						if((medsize > magic_number) and	(c.size() > medsize * (1. + tolerance_sup)))
							ret = validate_remove(c);
						return ret;
					}
				),
				clusters.end()
			);
			clusters.shrink_to_fit();	
		}
		
		for (std::size_t i = 0; i < clusters.size() ; ++i)
		{
			//Update observation
			for(const auto& id : clusters[i])
			{
				CheckerBoardObservation o = obs[id]; 
				o.cluster = i;
				filtered.emplace_back(o);		
			}
		}
			
		GUI(
			v::Palette<int> palette;
			for (const auto& o : filtered)
			{				
				RENDER_DEBUG_2D(
		  			Viewer::context().layer(Viewer::layer())
		  				.name("clusters_filtered")
		  				.pen_color(palette[o.cluster]).pen_width(8),
		  			P2D{o[0], o[1]}
				);	
			}
		
			Viewer::update();
		);
	}
	
	filtered.shrink_to_fit();
	return filtered;
}
