#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/sensor.h"

#include "types.h"

namespace ttt
{
    template<> 
    struct Name<Sensor> {static std::string name(){return "Sensor";}};
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Sensor
////////////////////////////////////////////////////////////////////////////////////////////////////
    template<>
    struct Size<Sensor> { enum{value = 3}; };

    void apply_increment(Sensor& s, const double delta[Size<Sensor>::value], const Adl&);
    
    void apply_small_increment(Sensor& s, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(Sensor& s, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(Sensor& s, double h, const v::core::numeric_tag<2>&, const Adl&);

} // namespace lma
