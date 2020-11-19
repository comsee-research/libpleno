#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/mesh.h"
#include "geometry/sensor.h"
#include "geometry/pose.h"

#include "geometry/camera/plenoptic.h"

#include "geometry/object/checkerboard.h"

struct BlurRadiusReprojectionError
{
	using ErrorType 	= Eigen::Matrix<double, 1, 1>; //rho
	using MLA_t			= GridMesh3D;
	
	const PlenopticCamera& pcm;
	const CheckerBoard &checkerboard;
    const BAPObservation& observation;

    bool operator()( 
    	const Pose& camera_pose,
    	const Pose& mla_pose,
		const MLA_t& mla,
		const FocalLength& f,
    	const Sensor& sensor,
    	const ThinLensCamera& tcm, 
    	const Distortions& distortions,
    	ErrorType& error
    ) const;
};

namespace ttt
{
	template<> 
	struct Name<BlurRadiusReprojectionError> 
	{ 
		static std::string name(){ return "BlurRadiusReprojectionError"; } 
	};
} // namespace ttt
