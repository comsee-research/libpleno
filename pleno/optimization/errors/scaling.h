#pragma once

#include <libv/lma/lma.hpp>

#include <map>

#include "types.h"

#include "geometry/camera/plenoptic.h" //PlenopticCamera
#include "geometry/object/checkerboard.h"
#include "geometry/observation.h"

#include "geometry/depth/depthmap.h"

#include "processing/tools/functions.h"

template <typename FunctionType>
struct ScalingCostError
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //mbe
	
	const PlenopticCamera& mfpc;
	const CheckerBoard& scene;
	
	const DepthMap& dm;
	std::unordered_map<Index /* cluster index */, BAPObservations> clusters;
	
    ScalingCostError(
    	const PlenopticCamera& mfpc_,
    	const CheckerBoard& scene_, 
    	const DepthMap& dm_,
    	const BAPObservations& observations
    );

    bool operator()( 
    	const FunctionType& f,
    	ErrorType& error
    ) const;
};

using LinearScalingCostError 	= ScalingCostError<LinearFunction>;
using QuadraticScalingCostError = ScalingCostError<QuadraticFunction>;

namespace ttt
{
	template<> 
	struct Name<LinearScalingCostError> 
	{ 
		static std::string name(){ return "LinearScalingCostError"; } 
	};
	
	template<> 
	struct Name<QuadraticScalingCostError> 
	{ 
		static std::string name(){ return "QuadraticScalingCostError"; } 
	};
} // namespace ttt
