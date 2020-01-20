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

template<typename CameraModel_t>
struct CornerReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 2, 1>; //u, v
	using MLA_t		= GridMesh3D;
	
	const CameraModel_t& pcm;
	const CheckerBoard &checkerboard;
    const BAPObservation& observation;

    bool operator()( 
    	const Pose& camera_pose,
    	const Pose& mla_pose,
		const MLA_t& mla,
    	const Sensor& sensor,
    	const ThinLensCameraModel& tcm, 
    	const Distortions& distortions,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<CornerReprojectionError<MultiFocusPlenopticCamera>> 
	{ 
		static std::string name(){ return "CornerReprojectionError<MFPC>"; } 
	};
} // namespace ttt
