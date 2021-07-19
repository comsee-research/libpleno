#pragma once

#include <libv/lma/lma.hpp>

#include "geometry/distortions.h"

#include "types.h"

#define USE_DEPTH_DISTORTIONS 0

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
#if defined(USE_DEPTH_DISTORTIONS) && USE_DEPTH_DISTORTIONS
	template<>
	struct Size<Distortions> { enum{value = 8}; };
#else
	template<>
	struct Size<Distortions> { enum{value = 5}; };
#endif

    void apply_increment(Distortions& d, const double delta[Size<Distortions>::value], const Adl&);
    
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<0>&, const Adl&);
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<1>&, const Adl&);
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<2>&, const Adl&);
    
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<3>&, const Adl&);
    void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<4>&, const Adl&);
    
#if defined(USE_DEPTH_DISTORTIONS) && USE_DEPTH_DISTORTIONS    
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<5>&, const Adl&);
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<6>&, const Adl&);
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<7>&, const Adl&);
#endif
} // namespace lma
