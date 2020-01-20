#pragma once

#include <iostream>

#include <libv/core/serialization/archives/base.hpp> //OutputArchive, InputArchive

struct InternalParameters {
	double m;
	double c[3];
	double c_prime[3];		
	double kappa;			//metric distance between 2 µ-Images
	double kappa_approx;	//metric distance between 2 µ-Lenses
	double N; 				//f-number
	double scale; 			//pixel2metric
	
	double radius(int t) const { return (m / N + c[t]) / scale; }
};

std::ostream& operator<<(std::ostream& os, const InternalParameters& params);
void save(v::OutputArchive& archive, const InternalParameters& params);
void load(v::InputArchive& archive, InternalParameters& params);

