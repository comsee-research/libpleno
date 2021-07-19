#pragma once

#include <iostream>

#include "io/archive.h"
#include "io/printer.h"

struct InternalParameters {
	double m;
	std::vector<double> q;
	std::vector<double> q_prime;
			
	double dc;			//metric distance between 2 µ-Images
	double dC;			//metric distance between 2 µ-Lenses
	double lambda;		//approximate ratio between dc and dC lambda=~0.9931
	
	double N; 				//f-number
	double scale; 			//pixel2metric
	std::size_t I = 3ul;		//number of micro-lenses type
	
	double kappa = 1.; 		//blur proportionality coefficient
	
	double radius(int t) const {
		if (I == 0ul or t == -1) return 0.5 * dc / scale;
		 
		DEBUG_ASSERT((std::size_t(t) < I), "Can't access type = " << t << "/" << I); 
		return (m / N + q[t]) / scale; 
	}
};

std::ostream& operator<<(std::ostream& os, const InternalParameters& params);
void save(v::OutputArchive& archive, const InternalParameters& params);
void load(v::InputArchive& archive, InternalParameters& params);

