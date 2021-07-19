#include "distortions.h"

namespace lma
{
	////////////////////////////////////////////////////////////////////////////////////////////////////
    // On Distortions
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void apply_increment(Distortions& d, const double delta[Size<Distortions>::value], const lma::Adl&)
	{
		d.radial()[0] += delta[0];
		d.radial()[1] += delta[1];
		d.radial()[2] += delta[2];
		
		d.tangential()[0] += delta[3];
		d.tangential()[1] += delta[4];
		
#if defined(USE_DEPTH_DISTORTIONS) && USE_DEPTH_DISTORTIONS	 
		d.depth()[0] += delta[5];
		d.depth()[1] += delta[6];
		d.depth()[2] += delta[7];
#endif
	}

	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<0>&, const lma::Adl&)
	{ 
		d.radial()[0] += h; 
	}
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<1>&, const lma::Adl&)
	{ 
		d.radial()[1] += h; 
	}
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<2>&, const lma::Adl&)
	{ 
		d.radial()[2] += h; 
	}

	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<3>&, const lma::Adl&)
	{ 
		d.tangential()[0] += h; 
	}
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<4>&, const lma::Adl&)
	{ 	
		d.tangential()[1] += h; 
	}
#if defined(USE_DEPTH_DISTORTIONS) && USE_DEPTH_DISTORTIONS
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<5>&, const lma::Adl&)
	{ 
		d.depth()[0] += h; 
	}
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<6>&, const lma::Adl&)
	{ 
		d.depth()[1] += h; 
	}
	void apply_small_increment(Distortions& d, double h, const v::core::numeric_tag<7>&, const lma::Adl&)
	{ 
		d.depth()[2] += h; 
	}
#endif
}
