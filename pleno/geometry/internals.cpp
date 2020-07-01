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
	for(const auto& qi : params.q)
		os << "\tq["<<i++<<"] = " << qi << std::endl;
	i=0;
	for(const auto& qpi : params.q_prime)	
		os << "\tq'["<<i++<<"] = " << qpi << std::endl;
		
	os	<< "\tdc = " << params.dc << std::endl
		<< "\tlambda = " << params.lambda << std::endl
		<< "\tdC = " << params.dC << std::endl
		<< "\tN = " << params.N << std::endl
		<< "\tI = " << params.I << std::endl
		<< "\tkappa = " << params.kappa << std::endl
		<< "};" ;
		
	return os;
}

void save(v::OutputArchive& archive, const InternalParameters& params)
{
    archive
		("m", params.m)
		("scale", params.scale)
		("q", params.q)
		("q_prime", params.q_prime)
		("dc", params.dc)
		("dC", params.dC)
		("N", params.N)
		("I", params.I)
		("kappa", params.kappa)
		("lambda", params.lambda);
}

void load(v::InputArchive& archive, InternalParameters& params)
{
    archive
		("m", params.m)
		("scale", params.scale)
		("q", params.q)
		("q_prime", params.q_prime)
		("dc", params.dc)
		("dC", params.dC)
		("N", params.N)
		("I", params.I)
		("kappa", params.kappa)
		("lambda", params.lambda);
}

