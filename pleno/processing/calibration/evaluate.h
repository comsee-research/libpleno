#pragma once

#include <type_traits>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/reprojection.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/object/checkerboard.h"

#include "geometry/depth/depthmap.h"
#include "geometry/depth/pointcloud.h"

#include "processing/tools/error.h"
#include "processing/tools/functions.h"

#include "io/printer.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline RMSE compute_rmse_center(
	const PlenopticCamera& model,
	const MICObservations& centers
)
{
	RMSE rmse;
	for(const auto& c : centers)
	{	
		const P2D p = reproject_miccenter(model, c);
		const P2D residual{P2D{c[0], c[1]} - p};
		
		rmse.add(residual);
	}
	return rmse;
}

template<typename Observations>
RMSE compute_rmse_corner(
	const PlenopticCamera& model,
	const CheckerBoard& grid,
	const Observations& ob
)
{
	RMSE rmse;
	for(const auto& o : ob)
	{	
		const P2D p = reproject_corner(model, model.pose(), grid, o, false);
		const P2D residual{P2D{o[0], o[1]} - p};
		
		rmse.add(residual);
	}
	return rmse;
}

template<typename Observations>
RMSE compute_rmse_radius(
	const PlenopticCamera& model,
	const CheckerBoard& grid,
	const Observations& ob
)
{
	RMSE rmse;
	if (model.focused() and std::is_same_v<Observations, BAPObservations>) {
		for(const auto& o : ob)
		{	
			const double p = reproject_radius(model, model.pose(), grid, o);
			const double residual = o[2] - p;
			
			rmse.add(residual);
		}
	}
	return rmse;
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<typename Observations>
void evaluate_rmse(
	const PlenopticCamera& mfpc,
	const CalibrationPoses& poses,
	const CheckerBoard& grid,
	const Observations& observations,
	const MICObservations& centers,
	bool  export_as_csv = true
)
{
	std::ofstream ofs;
	
	//Split observations according to frame
	std::unordered_map<Index /* frame index */, Observations> obs;
	for (const auto& ob : observations)
		obs[ob.frame].push_back(ob);
		
	auto model = mfpc;

	PRINT_DEBUG("\t\tERROR\t\tRMSE");
	PRINT_DEBUG("---------------------------------------------");
	if (export_as_csv)
	{	
		ofs.open("rmse-"+std::to_string(getpid())+".csv");
		if (!ofs.good())
			throw std::runtime_error(std::string("Cannot open file errors.csv"));
	
		ofs << "f,uv,rho,bap,center\n";
	}
	
	RMSE rmse_center = compute_rmse_center(model, centers);
	RMSE rmse_total = rmse_center;	
	
//For each frame 
	RMSE rmse_bap_all{0., 0}, rmse_corner_all{0., 0}, rmse_radius_all{0., 0};	
	for(auto & [f, o] : obs)
	{
		for(const auto& [p,f] : poses) if(f == o[0].frame) model.pose() = p;
		
		PRINT_DEBUG("Frame = " << f);
		
		RMSE rmse_corner = compute_rmse_corner(model, grid, o);
		PRINT_DEBUG("uv\t"<< (rmse_corner.sum()) << "\t\t" << rmse_corner.get());
		rmse_corner_all += rmse_corner;
		
		RMSE rmse_radius = compute_rmse_radius(model, grid, o);
		PRINT_DEBUG("rho\t"<< (rmse_radius.sum()) << "\t\t" << rmse_radius.get());
		rmse_radius_all += rmse_radius;
			
		RMSE rmse_bap = rmse_corner; rmse_bap.value += rmse_radius.value;	
		PRINT_DEBUG("bap\t"<< (rmse_bap.sum()) << "\t\t" << rmse_bap.get());
		rmse_bap_all += rmse_bap;	
		
		PRINT_DEBUG("---------------------------------------------");
		
		if (export_as_csv)
		{	
			std::ostringstream oss;	
			oss << f 					<< ","
				<< rmse_corner.get() 	<< ","
				<< rmse_radius.get() 	<< ","
				<< rmse_bap.get()		<< ","
				<< 0. 					<< "\n"; 
			ofs << oss.str();
		}
	}
	(rmse_total += rmse_corner_all) += rmse_radius_all;
	
	PRINT_DEBUG("MIC\t"<< (rmse_center.sum()) << "\t\t" << rmse_center.get());
	PRINT_DEBUG("UV\t"<< (rmse_corner_all.sum()) << "\t\t" << rmse_corner_all.get());
	PRINT_DEBUG("RHO\t"<< (rmse_radius_all.sum()) << "\t\t" << rmse_radius_all.get());
	PRINT_DEBUG("BAP\t"<< (rmse_bap_all.sum()) << "\t\t" << rmse_bap_all.get());
	PRINT_DEBUG("---------------------------------------------");
	PRINT_DEBUG("TOT.\t"<< (rmse_total.sum()) << "\t\t" << rmse_total.get());
	
	if (export_as_csv)
	{		
		std::ostringstream oss;	
		oss << -1 << ","
			<< rmse_corner_all.get() 	<< ","
			<< rmse_radius_all.get() 	<< ","
			<< rmse_bap_all.get()		<< ","
			<< rmse_center.get() 			<< "\n"; 
		
		ofs << oss.str();
		ofs.close();
	}
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
inline void evaluate_scale_error(
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, DepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations,
	const std::unordered_map<Index, Image>& pictures,
	bool  export_as_csv = true
) 
{
	v::Palette<int> palette;
	
	std::ofstream ofs;
	if (export_as_csv)
	{
		ofs.open(
		"	scale-error-" + std::to_string(getpid())+".csv"
		); 
		if (not ofs.good()) throw std::runtime_error(std::string("Cannot open file scale-error.csv"));
	
		std::ostringstream headercsv;
		headercsv << "f,v,z,mae,mbe,rmse\n";
	
		ofs << headercsv.str();
	}
	
	std::ostringstream oss;

	//for each depth map
	for (const auto& [frame, dm] : depthmaps)
	{
		//get image
		Image image = pictures.at(frame);
		
		//convert to pc
		const DepthMap mdm = dm.to_metric(mfpc);
		const PointCloud pc = PointCloud{mdm, mfpc, image};
		
		GUI(		
			display(frame, pc);		
			
			RENDER_DEBUG_2D(
				Viewer::context().layer(Viewer::layer()).name("Frame f = "+std::to_string(frame)), 
				image
			);
			Viewer::update();
		);			
		//split observations according to cluster index
		std::map<Index /* cluster index */, BAPObservations> clusters;
		BAPObservations obs; 
		try {
			obs = observations.at(frame);
		} 
		catch (const std::out_of_range& e) {
			PRINT_WARN("No observations for frame ("<< frame << ")");
			continue;
		}
		for(const auto& ob : obs) clusters[ob.cluster].push_back(ob);	
		
		std::map<Index /* cluster index */, P3D> centroids; 
		
		PRINT_INFO("Reprojecting centroid from observations and depthmap...");
		//for each cluster
		for(auto & [cluster, obs] : clusters)
		{
			//compute reprojected point 
			P3D centroid = P3D::Zero(); 
			double n = 0.;
			
			//for each observation
			for (const auto& ob : obs)
			{
				//get u,v
				const double u = std::floor(ob.u);
				const double v = std::floor(ob.v);
				
				//get depth
				double depth = dm.is_refined_map() ? dm.depth(u,v) : dm.depth(ob.k, ob.l);
				if (depth == DepthInfo::NO_DEPTH) continue;
				if (dm.is_virtual_depth()) depth = mfpc.v2obj(depth, ob.k, ob.l);				
				
				//get pixel
				const P2D pixel = P2D{ob.u, ob.v};
				
				//get ml indexes
				const P2D kl = mfpc.mi2ml(ob.k, ob.l); 
					
				//raytrace
				Ray3D ray; //in CAMERA frame
				if (mfpc.raytrace(pixel, kl[0], kl[1], ray))
				{
					//get depth plane
					PlaneCoefficients plane; plane << 0., 0., 1., -depth;
					
					//get position
					const P3D point = line_plane_intersection(plane, ray);
				
					//accumulate
					centroid += point; ++n;
				}			
			}
			
			if (n != 0.)
			{
				centroid /= n;
			
				PRINT_DEBUG("Frame ("<< frame <<"), node ("<< cluster<<") = " << scene.node(cluster).transpose() << ", centroid = " << centroid.transpose());
				centroids[cluster] = std::move(centroid);		
			}			
		}
		
		wait();		
	
		//compute distances and errors
		RMSE rmse{0., 0ul};
		MAE	 mae{0., 0ul};
		MBE	 mbe{0., 0ul};
		
		PRINT_INFO("Computing error...");
		for (auto it = centroids.cbegin(); it != centroids.cend(); ++it)
		{
			GUI(
				BAPObservations obs;
				mfpc.project(it->second, obs);
				for (const auto& o : obs)
				{
					RENDER_DEBUG_2D(
			  			Viewer::context().layer(Viewer::layer())
			  				.name("Reprojected BAP ("+std::to_string(frame)+")")
							.point_style(v::Cross)
			  				.pen_color(palette[o.cluster+1]).pen_width(2),
			  			Disk{P2D{o.u, o.v}, o.rho}
					);	
				}	
					
				RENDER_DEBUG_3D(
					Viewer::context(Viewer::Mode::m3D).layer(Viewer::layer(Viewer::Mode::m3D))
						.name("Centroid"),
					it->second,	5.
			  	);
			);			
			
			for (auto nit = std::next(it, 1); nit != centroids.cend(); ++nit)	
			{
				const double ref = (scene.node(it->first) - scene.node(nit->first)).norm();
				const double dist = (it->second - nit->second).norm();
				
				DEBUG_VAR(ref); DEBUG_VAR(dist);
				rmse.add(100. * (ref-dist) / ref);		
				mae.add(100. * (ref-dist) / ref);		
				mbe.add(100. * (ref-dist) / ref);					
			}
		}
		
		if (centroids.cbegin() != centroids.cend())
		{
			const double z = std::accumulate(centroids.cbegin(), centroids.cend(), 0., 
					[](double tz, const auto& c) { return tz + c.second.z(); }
				) / centroids.size();
			
			if (export_as_csv)
			{
				oss << frame << "," << mfpc.obj2v(z) << "," << z << "," 
					<< mae.get() << "," << mbe.get() << "," << rmse.get() << "\n";			
			}
			
			GUI(	
				Viewer::update(Viewer::Mode::m2D);
				Viewer::update(Viewer::Mode::m3D);
			);
			PRINT_INFO("Error of frame ("<< frame <<") = " << rmse.get() << " (rmse), " << mae.get() << " (mae), " << mbe.get() << " (mbe)");
		}
		else
		{
			PRINT_WARN("Can't compute error for frame ("<< frame << ")");
		}				
		
		wait();	
	}
	
	if (export_as_csv)
	{
		ofs << oss.str();		
		ofs.close();
	}
}
