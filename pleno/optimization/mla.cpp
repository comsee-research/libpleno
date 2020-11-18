#include "optimization.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On MLA
////////////////////////////////////////////////////////////////////////////////////////////////////
	void apply_increment(MLA& g, const double delta[Size<MLA>::value], const Adl&)
	{
		g.pitch()[0] += delta[0];
		g.pitch()[1] += delta[0];
		g.f(0) += delta[1];
		g.f(1) += delta[2];
		g.f(2) += delta[3];
	}

	void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<0>&, const Adl&)
	{
		g.pitch()[0] += h;
		g.pitch()[1] += h;
	}

	void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<1>&, const Adl&)
	{
		g.f(0) += h;
	}

	void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<2>&, const Adl&)
	{
		g.f(1) += h;
	}

	void apply_small_increment(MLA& g, double h, const v::core::numeric_tag<3>&, const Adl&)
	{
		g.f(2) += h;
	}
} // namespace lma
