#include "grid.h"

#include "io/printer.h"

bool GridReconstructionError::operator()(const MIA& g, const Pose2D& p, ErrorType& error) const
{
	const P2D c = g.nodeInWorld(observation.k, observation.l);
	
//	PRINT_DEBUG( 
//		"k,l = " << observation.k << ", " << observation.l << std::endl <<
//		"u,v = " << observation[0] << ", " << observation[1] << std::endl <<
//		"c = " << c	
//	);

    error[0] = observation[0] - c[0];
    error[1] = observation[1] - c[1];

    return true;
}

