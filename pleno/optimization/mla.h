#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/mla.h"

namespace ttt
{
    template<>
    struct Name<MLA>{ static std::string name(){ return "MicroLensesArray"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On MLA
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<MLA>{ enum{ value = 4 }; };

    void apply_increment(MLA& g, const double delta[Size<MLA>::value], const Adl&);

    void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<2>&, const Adl&);
    void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<3>&, const Adl&);
} // namespace lma
