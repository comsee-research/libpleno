#pragma once

#include "types.h"
#include "transformation.h"

////////////////////////////////////////////////////////////////////////////////
/*
 * Apply transformation
 * TODO: add a last (col,row) in order to remove the black borders
 */
bool warp(const Transformation& t, const Image& input, Image& output);

////////////////////////////////////////////////////////////////////////////////
struct GrayInterpolator {
    const Image& mask;

    bool is_valid(const P2D& pixel) const;

    double intensity(const Image& image, const P2D& pixel) const;

    // Computing nearest rounded values neighbors of a point
    P2DS neighbors(const P2D& pixel) const;

    double operator()(const Image& image, const P2D& pixel) const;
};
