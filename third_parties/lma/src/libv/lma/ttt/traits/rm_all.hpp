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

#ifndef __LMA_TTT_TRAITS_RM_ALL_HPP__
#define __LMA_TTT_TRAITS_RM_ALL_HPP__

#include <type_traits>

namespace ttt
{
  template<class T> using rm_all = std::decay<T>;
}

#endif