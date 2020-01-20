#pragma once

#include "types.h"

#include "geometry/transformation.h"

/**
 *  @Brief fit_template_on
 */
Transformation optimize(
    const Image& observation,
    const Image& intial_template,
    const Image& mask,
    double& final_residual
);

