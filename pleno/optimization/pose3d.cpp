#include "pose3d.h"

#include <libv/geometry/rotation.hpp> 

namespace lma 
{
////////////////////////////////////////////////////////////////////////////////////////////////////
    // On Poses
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(Pose& p, const double delta[Size<Pose>::value], const lma::Adl&)
{
    p.translation()[0] += delta[0];
    p.translation()[1] += delta[1];
    p.translation()[2] += delta[2];
    v::apply_rotation(p.rotation(), Eigen::Vector3d(delta[3], 0.0, 0.0));
    v::apply_rotation(p.rotation(), Eigen::Vector3d(0.0, delta[4], 0.0));
    v::apply_rotation(p.rotation(), Eigen::Vector3d(0.0, 0.0, delta[5]));
    
    //v::rotation_orthogonalize(p.rotation());
}

void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
{ 
	p.translation()[0] += h; 
}

void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<1>&, const lma::Adl&)
{ 
	p.translation()[1] += h; 
}

void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<2>&, const lma::Adl&)
{ 
	p.translation()[2] += h; 
}

void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<3>&, const lma::Adl&)
{ 
	v::apply_rotation(p.rotation(), Eigen::Vector3d(h, 0.0, 0.0)); 
}

void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<4>&, const lma::Adl&)
{ 
	v::apply_rotation(p.rotation(), Eigen::Vector3d(0.0, h, 0.0)); 
}

void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<5>&, const lma::Adl&)
{ 
	v::apply_rotation(p.rotation(), Eigen::Vector3d(0.0, 0.0, h)); 
}

}
