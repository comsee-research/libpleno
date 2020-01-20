#pragma once

#include <libv/lma/lma.hpp>
#include "geometry/pose.h"

namespace ttt
{
    template<>
    struct Name<Pose2D>{ static std::string name(){ return "Pose2D"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Poses
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<Pose2D>{ enum{ value = 3 }; };

    void apply_increment(Pose2D& p, const double delta[Size<Pose2D>::value], const Adl&);
    
    void apply_small_increment(Pose2D& p, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(Pose2D& p, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(Pose2D& p, double h, const v::core::numeric_tag<2>&, const Adl&);

} // namespace lma
