#include "internals.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const InternalParameters& params)
{	
	os 	<< "{" << std::endl
		<< "\tm = " << params.m << std::endl
		<< "\tscale = " << params.scale << std::endl;
		
	std::size_t i=0;	
	for(const auto& ci : params.c)
		os << "\tc["<<i++<<"] = " << ci << std::endl;
	i=0;
	for(const auto& cpi : params.c_prime)	
		os << "\tc'["<<i++<<"] = " << cpi << std::endl;
		
	os	<< "\tkappa = " << params.kappa << std::endl
		<< "\tkappa_approx = " << params.kappa_approx << std::endl
		<< "\tN = " << params.N << std::endl
		<< "\tI = " << params.I << std::endl
		<< "};" ;
		
	return os;
}

void save(v::OutputArchive& archive, const InternalParameters& params)
{
    archive
		("m", params.m)
		("scale", params.scale)
		("c", params.c)
		("c_prime", params.c_prime)
		("kappa", params.kappa)
		("kappa_approx", params.kappa_approx)
		("N", params.N)
		("I", params.I);
}

void load(v::InputArchive& archive, InternalParameters& params)
{
    archive
		("m", params.m)
		("scale", params.scale)
		("c", params.c)
		("c_prime", params.c_prime)
		("kappa", params.kappa)
		("kappa_approx", params.kappa_approx)
		("N", params.N)
		("I", params.I);
}

