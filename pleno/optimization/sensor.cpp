#include "sensor.h"

//#include <libv/geometry/rotation.hpp> 

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
    // On Sensor
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(Sensor& s, const double delta[Size<Sensor>::value], const lma::Adl&)
{
    s.pose().translation()[0] += delta[0];
    s.pose().translation()[1] += delta[1];
    s.pose().translation()[2] += delta[2];
}

void apply_small_increment(Sensor& s, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
{ 
	s.pose().translation()[0] += h; 
}

void apply_small_increment(Sensor& s, double h, const v::core::numeric_tag<1>&, const lma::Adl&)
{ 
	s.pose().translation()[1] += h; 
}

void apply_small_increment(Sensor& s, double h, const v::core::numeric_tag<2>&, const lma::Adl&)
{ 
	s.pose().translation()[2] += h; 
}

}
