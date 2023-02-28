/**

\file
\author Datta Ramadasan
//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================

*/

#ifndef __UTILS_SRC_TIME_TIME_HPP__
#define __UTILS_SRC_TIME_TIME_HPP__

#include <libv/core/time.hpp>

namespace utils {

//************************
//! Cette fonction lit le nombre de cycle processeur
//************************
static inline double read_cycles() {
  size_t hi, lo;
#if 0
  __asm __volatile ("rdtsc" : "=a" (lo), "=d" (hi));
#else
  hi = lo = 0;
#endif
  return double((long long)hi << 32 | lo);
}

static inline double now() {
  return v::now();
}
}//! eon utils

#endif // TIME_HPP
