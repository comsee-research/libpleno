#pragma once

#include "io/archive.h"

V_DEFINE_PROPERTIES(LinearFunctionConfig)
(
    V_DEFINE_PROPERTY(a, (double(1.)), 	"Slope coefficient")
    V_DEFINE_PROPERTY(b, (double(0.)),	"y-intercept coefficient")
)

V_DEFINE_PROPERTIES(QuadraticFunctionConfig)
(
    V_DEFINE_PROPERTY(a, (double(0.)), 	"Second order term coefficient (z²)")
    V_DEFINE_PROPERTY(b, (double(1.)),	"First order term coefficient (z)")
    V_DEFINE_PROPERTY(c, (double(0.)),	"Zeroth order term coefficient (1)")
)
