#include "rotation3d.h"

#include "processing/tools/rotation.h"

namespace lma 
{
////////////////////////////////////////////////////////////////////////////////////////////////////
    // On Rotation
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(Rotation& rot, const double delta[Size<Rotation>::value], const lma::Adl&)
{    
    apply_rotation(*(rot.R), Eigen::Vector3d(delta[0], delta[1], delta[2]));
    rotation_orthogonalize(*(rot.R));
}

void apply_small_increment(Rotation& rot, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
{ 
	apply_rotation(*(rot.R), Eigen::Vector3d(h, 0.0, 0.0)); 
}

void apply_small_increment(Rotation& rot, double h, const v::core::numeric_tag<1>&, const lma::Adl&)
{ 
	apply_rotation(*(rot.R), Eigen::Vector3d(0.0, h, 0.0)); 
}

void apply_small_increment(Rotation& rot, double h, const v::core::numeric_tag<2>&, const lma::Adl&)
{ 
	apply_rotation(*(rot.R), Eigen::Vector3d(0.0, 0.0, h)); 
}

}
