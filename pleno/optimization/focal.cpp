#include "optimization.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On FocalLength
////////////////////////////////////////////////////////////////////////////////////////////////////
	void apply_increment(FocalLength& f, const double delta[Size<FocalLength>::value], const Adl&)
	{
		f.f += delta[0];
	}

	void apply_small_increment(FocalLength& f, double h, const v::core::numeric_tag<0>&, const Adl&)
	{
		f.f += h;
	}
} // namespace lma
