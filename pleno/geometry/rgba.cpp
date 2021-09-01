#include "rgba.h"

//******************************************************************************
void save(v::OutputArchive& archive, const RGBA& color)
{
	archive
		("r", color.r)
		("g", color.g)
		("b", color.b)
		("a", color.a);
}

void load(v::InputArchive& archive, RGBA& color)
{
	archive
		("r", color.r)
		("g", color.g)
		("b", color.b)
		("a", color.a);
}
