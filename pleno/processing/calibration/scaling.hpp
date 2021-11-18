#pragma once

#include <type_traits>

#include "geometry/camera/plenoptic.h"
#include "geometry/object/checkerboard.h"
#include "geometry/observation.h"

#include "geometry/depth/depthmap.h"

#include "processing/tools/functions.h"

//optimization
#include "optimization/functions.h"
#include "optimization/errors/scaling.h" //ScalingCostError

//io
#include "io/printer.h"
#include "io/choice.h"

template <typename ScalingFunction>
void optimize(
	ScalingFunction& scaling, //LinearFunction& scaling,    
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, DepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
)
{			
	using Solver_t = lma::Solver<ScalingCostError<ScalingFunction>>;
	
	Solver_t solver{-1., 1500, 1.0 - 1e-19};

	//for each frame
	for (const auto& [frame, dm] : depthmaps)
	{
		if (auto it = observations.find(frame); it != observations.end())
		{
			const BAPObservations& bap = it->second;
			
			//add ref point to solver
			solver.add(
				ScalingCostError<ScalingFunction>{mfpc, scene, dm, bap},
				&scaling
			);
		}	
		else
		{
			PRINT_ERR("No functor to add for frame f = " << frame);
		}
	}
	
	solver.solve(lma::DENSE, lma::enable_verbose_output());
}

template <typename ScalingFunction>
void calibration_depthScaling(                        
	ScalingFunction& scaling,     
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, DepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
)
{		
//1) Init Parameters
	PRINT_INFO("=== Init Parameter");	
	ScalingFunction f;
	
//3) Run optimization
	PRINT_INFO("=== Run optimization");	
	optimize(f, mfpc, scene, depthmaps, observations);
	
	PRINT_INFO("=== Optimization finished! Results:");
	
	if constexpr (std::is_same_v<ScalingFunction, LinearFunction>)
	{
		PRINT_INFO("Scaling is f(z) = "<< f.a << " * z + " << f.b );
			
		scaling.a = f.a;
		scaling.b = f.b;
	} 
	else if constexpr (std::is_same_v<ScalingFunction, QuadraticFunction>)
	{	
		PRINT_INFO("Scaling is f(z) = "<< f.a << " * z² + " << f.b << " * z + " << f.c);
	
		scaling.a = f.a;
		scaling.b = f.b;
		scaling.c = f.c;
	}
	
	wait();
}
