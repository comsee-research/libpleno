#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

namespace ttt
{
    template<>
    struct Name<BlurProportionalityCoefficient>{ static std::string name(){ return "BlurProportionalityCoefficient"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On BlurProportionalityCoefficient
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<BlurProportionalityCoefficient>{ enum{ value = 1 }; };

    void apply_increment(BlurProportionalityCoefficient& kappa, const double delta[Size<BlurProportionalityCoefficient>::value], const Adl&);
    void apply_small_increment(BlurProportionalityCoefficient& kappa, const double h, const v::core::numeric_tag<0>&, const Adl&);
} // namespace lma
