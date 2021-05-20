#include "invdistortion.h"

bool InverseDistortionCostError::operator()( 
	const Distortions& invdistortions,
	ErrorType& error
) const
{    
    error.setZero();

	//undistort point
	P3D target = ref;
	distortions.apply(target);
	
	//distort point again
	invdistortions.apply(target);
	
	//compute distance
	error = (ref - target).head<3>();

    return true;
}
