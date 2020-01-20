#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/mesh.h"
#include "geometry/sensor.h"
#include "geometry/pose.h"

#include "geometry/camera/models.h"
#include "geometry/camera/mfpc.h"

#include "geometry/object/checkerboard.h"

struct BlurAwarePlenopticReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 4, 1>; //u, v, rho, penalty
	using MLA_t = GridMesh3D;
	
	const MultiFocusPlenopticCamera& pcm;
	const CheckerBoard &checkerboard;
    const BAPObservation& observation;
    const bool use_corner_only = false;
    const bool use_regularization = false;

    bool operator()( 
    	const Pose& camera_pose,
    	const Pose& mla_pose,
		const MLA_t& mla,
		const FocalLength& f,
    	const Sensor& sensor,
    	const ThinLensCameraModel& tcm, 
    	const Distortions& distortions,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<BlurAwarePlenopticReprojectionError> 
	{ 
		static std::string name(){ return "BlurAwarePlenopticReprojectionError"; } 
	};
} // namespace ttt
