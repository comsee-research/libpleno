#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/distortions.h"

#include "types.h"

struct InverseDistortionCostError
{	
	using ErrorType = Eigen::Matrix<double, 3, 1>; //euclidiean distance
	
	const Distortions& distortions;
	const P3D ref; //original point
    
    InverseDistortionCostError(const Distortions& dists, const P3D& p) : distortions{dists}, ref{p} { }
    
    bool operator()( 
    	const Distortions& invdistortions,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<InverseDistortionCostError> 
	{ 
		static std::string name(){ return "InverseDistortionCostError"; } 
	};
} // namespace ttt
