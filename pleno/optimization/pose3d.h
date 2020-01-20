#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/pose.h"

namespace ttt
{
    template<>
    struct Name<Pose>{ static std::string name(){ return "Pose"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Poses
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<Pose>{ enum{ value = 6 }; };

    void apply_increment(Pose& p, const double delta[Size<Pose>::value], const Adl&);
    
    void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<2>&, const Adl&);
    void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<3>&, const Adl&);
    void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<4>&, const Adl&);
    void apply_small_increment(Pose& p, double h, const v::core::numeric_tag<5>&, const Adl&);

} // namespace lma
