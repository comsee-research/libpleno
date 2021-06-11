#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/mesh.h"
#include "geometry/sensor.h"
#include "geometry/pose.h"

#include "geometry/camera/plenoptic.h"
#include "geometry/object/checkerboard.h"

struct CornerReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 2, 1>; //u, v
	
	using MLA_t		= GridMesh3D;
	using CameraModel_t = PlenopticCamera;	
	
	const CameraModel_t& pcm;
	const CheckerBoard& checkerboard;
    const CBObservation observation;

    bool operator()( 
    	const Pose& camera_pose,
    	const Pose& mla_pose,
		const MLA_t& mla,
    	const Sensor& sensor,
    	const ThinLensCamera& tcm, 
    	const Distortions& distortions,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<CornerReprojectionError> 
	{ 
		static std::string name(){ return "CornerReprojectionError"; } 
	};
} // namespace ttt
