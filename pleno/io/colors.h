#pragma once

#include <iostream>
#include <iomanip>

template<typename T>
inline std::string to_string(const T t)
{
    std::ostringstream strs;
    strs << t;
    return strs.str();
}

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
    return red(to_string(t));
}

template<typename T>
inline std::string green(const T t)
{
    return green(to_string(t));
}

template<typename T>
inline std::string yellow(const T t)
{
    return yellow(to_string(t));
}



