#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/mia.h"

namespace ttt
{
    template<>
    struct Name<MIA>{ static std::string name(){ return "MicroImagesArray"; } };
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On GridMesh2D
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<MIA>{ enum{ value = 2 }; };

    void apply_increment(MIA& g, const double delta[Size<MIA>::value], const Adl&);

    void apply_small_increment(MIA& g, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(MIA& g, double h, const v::core::numeric_tag<1>&, const Adl&);
} // namespace lma
