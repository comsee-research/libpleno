#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

namespace ttt
{
    template<>
    struct Name<Translation>{ static std::string name(){ return "Translation"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Translation
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<Translation>{ enum{ value = 3 }; };

    void apply_increment(Translation& trans, const double delta[Size<Translation>::value], const Adl&);
    
    void apply_small_increment(Translation& trans, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(Translation& trans, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(Translation& trans, double h, const v::core::numeric_tag<2>&, const Adl&);

} // namespace lma
