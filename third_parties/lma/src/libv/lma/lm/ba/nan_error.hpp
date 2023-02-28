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

#ifndef _OPTIMISATION2_BA_NAN_ERROR__
#define _OPTIMISATION2_BA_NAN_ERROR__

#include <stdexcept>

namespace lma
{
  struct NAN_ERROR : std::runtime_error
  {
    NAN_ERROR(const std::string str) : std::runtime_error( str) {}
  };

  struct INF_ERROR : std::runtime_error
  {
    INF_ERROR(const std::string str) : std::runtime_error( str) {}
  };

  struct ZeroOrInfiniteError : std::runtime_error
  {
    ZeroOrInfiniteError(const std::string str) : std::runtime_error( str) {}
  };
}

#endif
