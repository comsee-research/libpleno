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

#ifndef __TTT_TRAITS_FUNCTION_DISPATCH_HPP__
#define __TTT_TRAITS_FUNCTION_DISPATCH_HPP__

#include "../fusion/at.hpp"
#include <boost/fusion/include/at_c.hpp>
#include "functor_trait.hpp"
#include "to_ref.hpp"
#include <libv/lma/ttt/traits/int.hpp>

namespace ttt
{
  namespace detail
  {
    template<class F, class Residu, class T, class ... A>
    inline bool apply_dispatch(Int<0> const&, const F& f, Residu& residu, const T&, const A& ... a)
    {
      return f(a...,residu);
    }
    
    template<size_t I, class F, class Residu, class T, class ... A>
    inline bool apply_dispatch(Int<I> const&, const F& f, Residu& residu, const T& t, const A& ... a)
    {
      return apply_dispatch(Int<I-1>(), f, residu, t, ttt::to_ref(bf::at_c<I-1>(t)), a...);
    }
  }

  template<class F, class Residu, class Vector>
  inline bool call(const F& f, Residu& residu, const Vector& vector)
  {
    typedef decltype(&F::operator()) op;
    typedef FunctorTrait<op> FT;
    static_assert( boost::is_same< typename FT::Residu, Residu&>::value, " Residu doesn't match");
    return detail::apply_dispatch(Int<FT::size>(),f,residu,vector);
  }
}// eon

#endif
