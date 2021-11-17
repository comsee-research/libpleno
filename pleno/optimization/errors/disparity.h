#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/camera/plenoptic.h" //PlenopticCamera
#include "geometry/mia.h" //MicroImage

template <bool useBlur = false>
struct DisparityCostError_
{		
	using ErrorType = Eigen::Matrix<double, 1, 1>; //SAD
	
	enum BlurMethod : std::uint8_t { S_TRANSFORM = 0, GAUSSIAN_BLUR = 1, APPROX_GAUSSIAN_BLUR = 2 };	
			
	const MicroImage mii;
	const MicroImage mij;
	
	const PlenopticCamera& mfpc;
	
	const P2D at;
	const BlurMethod method;
	
    DisparityCostError_( 
    	const MicroImage& mii_, const MicroImage& mij_, 
    	const PlenopticCamera& mfpc_,
    	const P2D& at_ = {-1.,-1.},
    	BlurMethod method_ = BlurMethod::S_TRANSFORM
    );
    DisparityCostError_(const DisparityCostError_& o);
    DisparityCostError_(DisparityCostError_&& o);

    bool operator()( 
    	const VirtualDepth& depth,
    	ErrorType& error
    ) const;
    
//helper   
    P2D 	disparity(double v) const;
    double 	weight(double v) const;
    bool 	compute_at_pixel() const;
};

using DisparityCostError = DisparityCostError_<false>;
using BlurAwareDisparityCostError = DisparityCostError_<true>;

namespace ttt
{
	template<> 
	struct Name<DisparityCostError> 
	{ 
		static std::string name(){ return "DisparityCostError"; } 
	};
	
	template<> 
	struct Name<BlurAwareDisparityCostError> 
	{ 
		static std::string name(){ return "BlurAwareDisparityCostError"; } 
	};
} // namespace ttt
