#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

namespace ttt
{
    template<>
    struct Name<Rotation>{ static std::string name(){ return "Rotation"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Rotation
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<Rotation>{ enum{ value = 3 }; };

    void apply_increment(Rotation& rot, const double delta[Size<Rotation>::value], const Adl&);
    
    void apply_small_increment(Rotation& rot, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(Rotation& rot, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(Rotation& rot, double h, const v::core::numeric_tag<2>&, const Adl&);

} // namespace lma
