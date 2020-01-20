#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/camera/mfpc.h"
#include "geometry/object/checkerboard.h"


struct ExtrinsicsBlurAwarePlenopticReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 3, 1>; //u,v,rho
	
	const MultiFocusPlenopticCamera& pcm;
	const CheckerBoard &checkerboard;
    const BAPObservation& observation;
    const bool use_corner_only = false;

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
