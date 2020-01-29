#pragma once

static constexpr double sqrt2 = std::sqrt(2.);
static constexpr double isqrt2 = 1. / sqrt2 ;
static constexpr double i2sqrt2 = isqrt2 * 0.5;
	
constexpr std::size_t factorial(std::size_t n)
{
    return n > 0u ? n * factorial( n - 1 ) : 1u;
}

constexpr auto pow(auto x, std::size_t n)
{
    return n > 0u ? x + pow(x, n - 1 ) : x;
}
