#pragma once

#include <type_traits>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/reprojection.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/object/checkerboard.h"

#include "processing/tools/rmse.h"

#include "io/printer.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
RMSE compute_rmse_center(
	const PlenopticCamera& model,
	const MICObservations& centers
)
{
	RMSE rmse;
	for(const auto& c : centers)
	{	
		const auto p = reproject_miccenter(model, c);
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
		const auto p = reproject_corner(model, model.pose(), grid, o);
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
	if (model.I()>0u and std::is_same_v<Observations, BAPObservations>) {
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
	std::unordered_map<int /* frame index */, Observations> obs;
	for (const auto& ob : observations)
		obs[ob.frame].push_back(ob);
		
	auto model = mfpc;

	PRINT_DEBUG("\t\tERROR\t\tRMSE");
	PRINT_DEBUG("---------------------------------------------");
	if (export_as_csv)
	{	
		std::ofstream ofs("rmse-"+std::to_string(getpid())+".csv");
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
			
		RMSE rmse_bap = rmse_radius; rmse_bap.value += rmse_corner.value;	
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
	}
}
