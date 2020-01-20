#pragma once

#include <libv/lma/lma.hpp>

#include "mla.h"

namespace ttt
{
    template<>
    struct Name<FocalLength>{ static std::string name(){ return "FocalLength"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On FocalLength
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<FocalLength>{ enum{ value = 1 }; };

    void apply_increment(FocalLength& f, const double delta[Size<FocalLength>::value], const Adl&);
    void apply_small_increment(FocalLength& f, double h, const v::core::numeric_tag<0>&, const Adl&);
} // namespace lma
