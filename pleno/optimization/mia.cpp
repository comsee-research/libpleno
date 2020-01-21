#include "optimization.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////
// On MIA
////////////////////////////////////////////////////////////////////////////////////////////////////
void apply_increment(MIA& g, const double delta[Size<MIA>::value], const Adl&)
{
    g.edge_length()[0] += delta[0];
    g.edge_length()[1] += delta[1];
}

void apply_small_increment(MIA& g, double h, const v::core::numeric_tag<0>&, const Adl&)
{
    g.edge_length()[0] += h;
}

void apply_small_increment(MIA& g, double h, const v::core::numeric_tag<1>&, const Adl&)
{
    g.edge_length()[1] += h;
}

} // namespace lma
