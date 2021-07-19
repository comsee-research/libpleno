#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/camera/plenoptic.h"
#include "geometry/object/checkerboard.h"

struct ExtrinsicsCornerReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 2, 1>; //u,v
	
	const PlenopticCamera& pcm;
	const CheckerBoard &checkerboard;
    const CBObservation observation;
    const std::size_t nbobsofframe = 1;

    bool operator()( 
    	const Pose& camera_pose,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<ExtrinsicsCornerReprojectionError> 
	{ 
		static std::string name(){ return "ExtrinsicsCornerReprojectionError"; } 
	};
} // namespace ttt

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
struct ExtrinsicsBlurAwarePlenopticReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 3, 1>; //u,v,rho
	
	const PlenopticCamera& pcm;
	const CheckerBoard &checkerboard;
    const BAPObservation observation;
    const std::size_t nbobsofframe = 1;

    bool operator()( 
    	const Pose& camera_pose,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<ExtrinsicsBlurAwarePlenopticReprojectionError> 
	{ 
		static std::string name(){ return "ExtrinsicsBlurAwarePlenopticReprojectionError"; } 
	};
} // namespace ttt
