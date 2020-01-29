#include "thinlens.h"

namespace lma 
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On ThinLensCamera
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(ThinLensCamera& tcm, const double delta[Size<ThinLensCamera>::value], const lma::Adl&)
{
    tcm.focal() += delta[0];
}

void apply_small_increment(ThinLensCamera& tcm, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
{ 
	tcm.focal() += h; 
}

}
