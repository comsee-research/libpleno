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

#ifndef __MODULES_TTT_TRAITS_UNROLL2_HPP__
#define __MODULES_TTT_TRAITS_UNROLL2_HPP__

#include <cstddef>
#include "int.hpp"

namespace ttt
{
  template< std::size_t I, std::size_t J, std::size_t F> struct DoubleUnroll2
  {
    template<class Func> static void unroll(Func func)
    {
      func.template operator() <I,J> ();
      DoubleUnroll2<I+1,J,F>::unroll(func);
    }
  };

  template< std::size_t F, std::size_t J> struct DoubleUnroll2<F,J,F>
  {
    template<class Func> static void unroll(Func) { }
  };

  //! Static double unroll who emulate a double for at compile time
  template<std::size_t ID, std::size_t JD, std::size_t IF, std::size_t JF> struct DoubleUnroll
  {
    template<class Func> static void unroll(Func func)
    {
      DoubleUnroll2<ID,JD,IF>::unroll(func);
      DoubleUnroll<ID,JD+1,IF,JF>::unroll(func);
    }
  };

  //! specialization to end the recursion on indice J when JD = JF
  template<std::size_t ID, std::size_t JF, std::size_t IF> struct DoubleUnroll<ID,JF,IF,JF>
  {
    template<class Func> static void unroll(Func) { }
  };

  template<std::size_t I, std::size_t J, class Func> void double_unroll(Int<I> const&, Int<J> const&, Func func)
  {
    DoubleUnroll<0,0,I,J>::unroll(func);
  }
}


#endif
