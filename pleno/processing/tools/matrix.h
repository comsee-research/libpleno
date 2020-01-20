#pragma once

#include <array>
#include <cmath>
#include <iostream>

/*
 * @Brief index_to_colRow returns the (col,row) coords of a desired element of an orthogonal matrix
**/
inline std::array<size_t, 2> index_to_colRow(size_t matrix_width, size_t index)
{
    if (matrix_width <= 0)
        std::cerr << "Error: index_to_colRow: "
                  << "the matrix width has not a good value (" << matrix_width << ")."
                  <<  std::endl;

    std::array<size_t, 2> indices;
    indices[1] = index / matrix_width;
    indices[0] = std::fma(-float(indices[1]), float(matrix_width), float(index));

    return indices;
}

inline size_t colRow_to_index(size_t matrix_width, const std::array<size_t, 2>& coords)
{
    return std::fma(float(coords[1]), float(matrix_width), float(coords[0]));
}

inline size_t colRow_to_index(size_t matrix_width, size_t col, size_t row)
{
    return std::fma(float(row), float(matrix_width), float(col));
}
