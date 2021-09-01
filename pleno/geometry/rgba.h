#pragma once

#include "types.h"

#include "io/archive.h"

//******************************************************************************
//******************************************************************************
struct RGBA { Real r = 0., g = 0., b = 0., a = 255.; };
using Colors 				= AlignedVector<RGBA>;

//******************************************************************************
//******************************************************************************
void save(v::OutputArchive& archive, const RGBA& color);
void load(v::InputArchive& archive, RGBA& color);
