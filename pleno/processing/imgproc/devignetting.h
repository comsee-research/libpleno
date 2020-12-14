#pragma once

#include "types.h"

////////////////////////////////////////////////////////////////////////////////
void devignetting(const Image& raw, const Image& white, Image& unvignetted);
void devignetting_u16(const Image& raw, const Image& white, Image& unvignetted);

