#pragma once

#include "types.h"

//******************************************************************************
//******************************************************************************
struct RGBA { Real r = 0., g = 0., b = 0., a = 255.; };
using Colors 				= AlignedVector<RGBA>;

//******************************************************************************
//******************************************************************************
inline void save(v::OutputArchive& archive, const RGBA& color)
{
	archive
		("r", color.r)
		("g", color.g)
		("b", color.b)
		("a", color.a);
}

inline void load(v::InputArchive& archive, RGBA& color)
{
	archive
		("r", color.r)
		("g", color.g)
		("b", color.b)
		("a", color.a);
}
