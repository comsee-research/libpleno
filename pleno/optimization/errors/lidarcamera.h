#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/camera/plenoptic.h"
#include "geometry/object/constellation.h"

struct LidarCameraCornerReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 2, 1>; //u,v
	
	const PlenopticCamera& pcm;
	const PointsConstellation &constellation;
    const CBObservation observation;

    bool operator()( 
    	const Pose& pose,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<LidarCameraCornerReprojectionError> 
	{ 
		static std::string name(){ return "LidarCameraCornerReprojectionError"; } 
	};
} // namespace ttt

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
struct LidarCameraBlurAwarePlenopticReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 3, 1>; //u,v,rho
	
	const PlenopticCamera& pcm;
	const PointsConstellation &constellation;
    const BAPObservation observation;

    bool operator()( 
    	const Pose& pose,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<LidarCameraBlurAwarePlenopticReprojectionError> 
	{ 
		static std::string name(){ return "LidarCameraBlurAwarePlenopticReprojectionError"; } 
	};
} // namespace ttt
