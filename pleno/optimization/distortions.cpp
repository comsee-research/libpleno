#include "distortions.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
    // On Distortions
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(Distortions& d, const double delta[Size<Distortions>::value], const lma::Adl&)
{
    d.radial()[0] += delta[0];
    d.radial()[1] += delta[1];
    d.radial()[2] += delta[2];
    d.tangential()[0] += delta[3];
    d.tangential()[1] += delta[4];
}

void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
{ 
	d.radial()[0] += h; 
}

void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<1>&, const lma::Adl&)
{ 
	d.radial()[1] += h; 
}

void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<2>&, const lma::Adl&)
{ 
	d.radial()[2] += h; 
}

void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<3>&, const lma::Adl&)
{ 
	d.tangential()[0] += h; 
}

void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<4>&, const lma::Adl&)
{ 	
	d.tangential()[1] += h; 
}

}
