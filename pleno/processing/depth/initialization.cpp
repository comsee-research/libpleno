#include "initialization.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::pair<double,double> initialize_min_max_distance(const PlenopticCamera& mfpc)
{
	const double F =  mfpc.focal();
	const double nearfocusd = 20. * F; 
	const double farfocusd = 100. * F; 
	const double h = mfpc.distance_focus();
	
	double mind, maxd;
	
	if(h < nearfocusd) //short distances
	{
		maxd = mfpc.distance_focus() * 1.2;
		mind = 4. * std::ceil(F); //8. * std::ceil(F); //
	}
	else if (h <= farfocusd) //middle distances
	{
		maxd = mfpc.distance_focus() * 2.;
		mind = 6. * std::ceil(F); //8. * std::ceil(F); //
	}
	else //far distances
	{
		maxd = farfocusd;
		mind = 8. * std::ceil(F); 
	}
	
	return {mind, maxd};
}
