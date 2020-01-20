#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/pose.h"
#include "geometry/mia.h"
#include "geometry/observation.h"

struct GridReconstructionError
{
	using ErrorType = Eigen::Matrix<double, 2, 1>;
	
    const MICObservation& observation;

    bool operator()(const MIA& g, const Pose2D& p, ErrorType& error) const;
};

namespace ttt
{
template<> struct Name<GridReconstructionError> { static std::string name(){ return "GridReconstructionError"; } };
} // namespace ttt
