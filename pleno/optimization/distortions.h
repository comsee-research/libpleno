#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/distortions.h"

#include "types.h"

namespace ttt
{
    template<> 
    struct Name<Distortions> {static std::string name(){return "Distortions";}};
} // namespace ttt

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On Distortions
////////////////////////////////////////////////////////////////////////////////////////////////////
	template<>
	struct Size<Distortions> { enum{value = 7}; };
	
    void apply_increment(Distortions& d, const double delta[Size<Distortions>::value], const Adl&);
    
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<2>&, const Adl&);
    
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<3>&, const Adl&);
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<4>&, const Adl&);
    
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<5>&, const Adl&);
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<6>&, const Adl&);
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<7>&, const Adl&);

} // namespace lma
