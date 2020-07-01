#include "optimization.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On BlurProportionalityCoefficient
////////////////////////////////////////////////////////////////////////////////////////////////////
	void apply_increment(BlurProportionalityCoefficient& kappa, const double delta[Size<BlurProportionalityCoefficient>::value], const Adl&)
	{
		kappa.kappa += delta[0];
	}

	void apply_small_increment(BlurProportionalityCoefficient& kappa, const double h, const v::core::numeric_tag<0>&, const Adl&)
	{
		kappa.kappa += h;
	}
} // namespace lma
