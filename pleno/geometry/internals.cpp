#include "internals.h"

//******************************************************************************
//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const InternalParameters& params)
{	
	os 	<< "{" << std::endl
		<< "\tm = " << params.m << std::endl
		<< "\tscale = " << params.scale << std::endl
		<< "\tc[0] = " << params.c[0] << std::endl
		<< "\tc[1] = " << params.c[1] << std::endl
		<< "\tc[2] = " << params.c[2] << std::endl
		<< "\tc'[0] = " << params.c_prime[0] << std::endl
		<< "\tc'[1] = " << params.c_prime[1] << std::endl
		<< "\tc'[2] = " << params.c_prime[2] << std::endl
		<< "\tkappa = " << params.kappa << std::endl
		<< "\tkappa_approx = " << params.kappa_approx << std::endl
		<< "\tN = " << params.N << std::endl
		<< "};" ;
		
	return os;
}

void save(v::OutputArchive& archive, const InternalParameters& params)
{
    archive
		("m", params.m)
		("scale", params.scale)
		("c_0", params.c[0])
		("c_1", params.c[1])
		("c_2", params.c[2])
		("c_prime_0", params.c_prime[0])
		("c_prime_1", params.c_prime[1])
		("c_prime_2", params.c_prime[2])
		("kappa", params.kappa)
		("kappa_approx", params.kappa_approx)
		("N", params.N);
}
void load(v::InputArchive& archive, InternalParameters& params)
{
    archive
		("m", params.m)
		("scale", params.scale)
		("c_0", params.c[0])
		("c_1", params.c[1])
		("c_2", params.c[2])
		("c_prime_0", params.c_prime[0])
		("c_prime_1", params.c_prime[1])
		("c_prime_2", params.c_prime[2])
		("kappa", params.kappa)
		("kappa_approx", params.kappa_approx)
		("N", params.N);
}

