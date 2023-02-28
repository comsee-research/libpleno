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

#ifndef __OPTIMISATION2_FUNCTION_DERIVATIVE_ANALYTICAL_DERIVATIVE_HPP__
#define __OPTIMISATION2_FUNCTION_DERIVATIVE_ANALYTICAL_DERIVATIVE_HPP__

#include <libv/lma/ttt/traits/int.hpp>
#include <libv/lma/ttt/traits/to_ref.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/fusion/at.hpp>
#include <libv/lma/global.hpp>

namespace lma
{
  template<class Tag> struct AnalyticalDerivator
  {
    template<class Fun, class Result, class Tuple, size_t I, size_t F, class ... R> static void _unroll(const Fun& fun, ttt::Int<I> const&, ttt::Int<F> const&, Result& result, const Tuple& tuple, R& ... r)
    {
      _unroll(fun,ttt::Int<I+1>(),ttt::Int<F>(),result,tuple,bf::at_c<F-1-I>(result).second,r...);
    }
    
    template<class Fun, size_t I, size_t F, class Tuple, class ... R> static void _unroll2(const Fun& fun, ttt::Int<I> const&, ttt::Int<F> const&, const Tuple& tuple, R& ... r)
    {
      _unroll2(fun,ttt::Int<I+1>(),ttt::Int<F>(),tuple,ttt::to_ref(bf::at_c<F-1-I>(tuple)),r...);
    }
    
    template<class Fun, size_t F, class Tuple, class ... R> static void _unroll2(const Fun& fun, ttt::Int<F> const&, ttt::Int<F> const&, const Tuple&, R& ... r)
    {
      fun.analytical_derivative(r...);
    }
    
    template<class Fun, size_t F, class Result, class Tuple, class ... R> static void _unroll(const Fun& fun, ttt::Int<F> const&, ttt::Int<F> const&, Result&, const Tuple& tuple, R& ... r)
    {
      _unroll2(fun,ttt::Int<0>(),ttt::Int<F>(),tuple,r...);
    }
    
    template<class Fonctor, class Tuple, class Jacob>
    static void derive(const Fonctor& fonctor, const Tuple& tuple, Jacob& result)
    {
      _unroll(fonctor.functor,ttt::Int<0>(),ttt::Int<mpl::size<Tuple>::value>(),result,tuple);
    }
  };
}

namespace ttt
{
  template<class Tag> struct Name<lma::AnalyticalDerivator<Tag>> { static std::string name(){ return "AnalyticalDerivator<" + ttt::name<Tag>() << ">"; } };
}

#endif
