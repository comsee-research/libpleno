#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/observation.h"
#include "geometry/mesh.h"
#include "geometry/sensor.h"
#include "geometry/pose.h"

#include "geometry/camera/plenoptic.h"

struct MicroImageCenterReprojectionError
{
	using ErrorType = Eigen::Matrix<double, 2, 1>;
	
	using MLA_t = GridMesh3D;
	
	const PlenopticCamera& pcm;
    const MICObservation& observation;
    
    bool operator()(const Pose& p, const MLA_t& g, const Sensor& sensor, ErrorType& error) const;
};

namespace ttt
{
	template<> 
	struct Name<MicroImageCenterReprojectionError> 
	{ 
		static std::string name(){ return "MicroImageCenterReprojectionError"; } 
	};
} // namespace ttt

using MICReprojectionError = MicroImageCenterReprojectionError;
