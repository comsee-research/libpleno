#pragma once

#include <array>
#include <cmath>
#include <iostream>

#include "io/printer.h"

/*
 * @Brief index_to_colRow returns the (col,row) coords of a desired element of an orthogonal matrix
**/
inline std::array<size_t, 2> index_to_colRow(size_t matrix_width, size_t index)
{
    DEBUG_ASSERT(matrix_width != 0, "Wrong width given during conversion of index to (col,row)");

    std::array<size_t, 2> indices;
    indices[1] = index / matrix_width;
    indices[0] = std::fma(-double(indices[1]), double(matrix_width), double(index));

    return indices;
}

inline size_t colRow_to_index(size_t matrix_width, const std::array<size_t, 2>& coords)
{
    return std::fma(double(coords[1]), double(matrix_width), double(coords[0]));
}

inline size_t colRow_to_index(size_t matrix_width, size_t col, size_t row)
{
    return std::fma(double(row), double(matrix_width), double(col));
}
