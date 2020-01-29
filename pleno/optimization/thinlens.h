#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/camera/thinlens.h"

namespace ttt
{
    template<> 
    struct Name<ThinLensCamera> {static std::string name(){return "ThinLensCamera";}};
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On ThinLensCameraModel
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<ThinLensCamera> { enum{value = 1}; };

    void apply_increment(ThinLensCamera& tcm, const double delta[Size<ThinLensCamera>::value], const Adl&);
    
    void apply_small_increment(ThinLensCamera& s, double h, const v::core::numeric_tag<0>&, const Adl&);

} // namespace lma
