#include "translation3d.h"

namespace lma 
{
////////////////////////////////////////////////////////////////////////////////////////////////////
    // On Translation
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(Translation& trans, const double delta[Size<Translation>::value], const lma::Adl&)
{
    (*(trans.t))[0] += delta[0];
    (*(trans.t))[1] += delta[1];
    (*(trans.t))[2] += delta[2];
}

void apply_small_increment(Translation& trans, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
{ 
	(*(trans.t))[0] += h; 
}

void apply_small_increment(Translation& trans, double h, const v::core::numeric_tag<1>&, const lma::Adl&)
{ 
	(*(trans.t))[1] += h; 
}

void apply_small_increment(Translation& trans, double h, const v::core::numeric_tag<2>&, const lma::Adl&)
{ 
	(*(trans.t))[2] += h; 
}

}
