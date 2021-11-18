#include "optimization.h"

namespace lma
{
////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_increment(LinearFunction& f, const double delta[Size<LinearFunction>::value], const Adl&)
{
	f.a += delta[0];
	f.b += delta[1];
}
void apply_small_increment(LinearFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&)
{
	f.a += h;
}
void apply_small_increment(LinearFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&)
{
	f.b += h;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_increment(QuadraticFunction& f, const double delta[Size<QuadraticFunction>::value], const Adl&)
{
	f.a += delta[0];
	f.b += delta[1];
	f.c += delta[2];
}
void apply_small_increment(QuadraticFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&)
{
	f.a += h;
}
void apply_small_increment(QuadraticFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&)
{
	f.b += h;
}
void apply_small_increment(QuadraticFunction& f, const double h, const v::core::numeric_tag<2>&, const Adl&)
{
	f.c += h;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_increment(CubicFunction& f, const double delta[Size<CubicFunction>::value], const Adl&)
{
	f.a += delta[0];
	f.b += delta[1];
	f.c += delta[2];
	f.d += delta[3];
}
void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&)
{
	f.a += h;
}
void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&)
{
	f.b += h;
}
void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<2>&, const Adl&)
{
	f.c += h;
}
void apply_small_increment(CubicFunction& f, const double h, const v::core::numeric_tag<3>&, const Adl&)
{
	f.d += h;
}
////////////////////////////////////////////////////////////////////////////////////////////////////

void apply_increment(QuarticFunction& f, const double delta[Size<QuarticFunction>::value], const Adl&)
{
	f.a += delta[0];
	f.b += delta[1];
	f.c += delta[2];
	f.d += delta[3];
	f.e += delta[4];
}
void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<0>&, const Adl&)
{
	f.a += h;
}
void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<1>&, const Adl&)
{
	f.b += h;
}
void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<2>&, const Adl&)
{
	f.c += h;
}
void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<3>&, const Adl&)
{
	f.d += h;
}
void apply_small_increment(QuarticFunction& f, const double h, const v::core::numeric_tag<4>&, const Adl&)
{
	f.e += h;
}
} // namespace lma
