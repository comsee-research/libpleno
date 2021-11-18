#include "calibration.h"

//optimization
#include "optimization/functions.h"
#include "optimization/errors/scaling.h" //ScalingCostError

//io
#include "io/printer.h"
#include "io/choice.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
template<>
void calibration_depthScaling<LinearFunction>(                        
	LinearFunction& scaling,  
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, DepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
);

template<>
void calibration_depthScaling<QuadraticFunction>(                        
	QuadraticFunction& scaling,
	const PlenopticCamera& mfpc, const CheckerBoard& scene,
	const std::unordered_map<Index, DepthMap>& depthmaps,
	const std::unordered_map<Index, BAPObservations>& observations
);
