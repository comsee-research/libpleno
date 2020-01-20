#pragma once

#include <libv/lma/lma.hpp>

#include "types.h"

#include "geometry/camera/models.h"

namespace ttt
{
    template<> 
    struct Name<ThinLensCameraModel> {static std::string name(){return "ThinLensCameraModel";}};
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On ThinLensCameraModel
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<ThinLensCameraModel> { enum{value = 1}; };

    void apply_increment(ThinLensCameraModel& tcm, const double delta[Size<ThinLensCameraModel>::value], const Adl&);
    
    void apply_small_increment(ThinLensCameraModel& s, double h, const v::core::numeric_tag<0>&, const Adl&);

} // namespace lma
