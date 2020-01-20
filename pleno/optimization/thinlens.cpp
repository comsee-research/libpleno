#include "thinlens.h"

namespace lma 
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On ThinLensCameraModel
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(ThinLensCameraModel& tcm, const double delta[Size<ThinLensCameraModel>::value], const lma::Adl&)
{
    tcm.f() += delta[0];
}

void apply_small_increment(ThinLensCameraModel& tcm, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
{ 
	tcm.f() += h; 
}

}
