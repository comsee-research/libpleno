#include "optimization.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On GridMesh3D
////////////////////////////////////////////////////////////////////////////////////////////////////
	void apply_increment(GridMesh3D& g, const double delta[Size<GridMesh3D>::value], const Adl&)
	{
		g.pitch()[0] += delta[0];
		g.pitch()[1] += delta[0];
	}

	void apply_small_increment(GridMesh3D& g, double h, const v::core::numeric_tag<0>&, const Adl&)
	{
		g.pitch()[0] += h;
		g.pitch()[1] += h;
	}
} // namespace lma
