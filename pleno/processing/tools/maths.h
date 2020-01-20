#pragma once

constexpr std::size_t factorial(std::size_t n)
{
    return n > 0u ? n * factorial( n - 1 ) : 1u;
}

constexpr auto pow(auto x, std::size_t n)
{
    return n > 0u ? x + pow(x, n - 1 ) : x;
}
