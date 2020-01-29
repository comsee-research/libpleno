#pragma once

#include <iostream>
#include <iomanip>

inline std::string red(const std::string s)
{
    return "\033[31m" + s + "\033[0m";
}

inline std::string light_red(const std::string s)
{
    return "\033[92m" + s + "\033[0m";
}

inline std::string green(const std::string s)
{
    return "\033[32m" + s + "\033[0m";
}

inline std::string yellow(const std::string s)
{
    return "\033[33m" + s + "\033[0m";
}

template<typename T>
inline std::string red(const T t)
{
    return red(std::to_string(t));
}

template<typename T>
inline std::string green(const T t)
{
    return green(std::to_string(t));
}

template<typename T>
inline std::string yellow(const T t)
{
    return yellow(std::to_string(t));
}



