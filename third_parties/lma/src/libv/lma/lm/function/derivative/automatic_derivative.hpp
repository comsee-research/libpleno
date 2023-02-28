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

#ifndef __LIBV_LMA_OPT2_FUNCTION_DERIVATIVE_AUTOMATIC_DERIVATIE_HPP__
#define __LIBV_LMA_OPT2_FUNCTION_DERIVATIVE_AUTOMATIC_DERIVATIE_HPP__

#include <libv/lma/ttt/traits/int.hpp>
#include <libv/lma/ttt/traits/to_ref.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/ttt/traits/unroll2.hpp>
#include <libv/lma/ttt/fusion/at.hpp>
#include <libv/lma/numeric/ad/ct/adct.hpp>
#include <libv/lma/lm/trait/size.hpp>
#include <libv/lma/lm/container/container.hpp>
#include <libv/lma/lm/container/adct_eigen.hpp>
#include <libv/lma/lm/container/adct_toon.hpp>
#include <libv/lma/lm/function/function.hpp>

namespace std
{
  template<int I, int S, class Float, class Tag> const adct::Ad<Float,S,Tag>& get(const adct::Ad<Float,S,Tag>& a)
  {
    static_assert( I==0, "std::get<adct::Ad<>> : I==0");
    return a;
  }
}

namespace lma
{
  template<class Float, int N, class Tag> struct Size<adct::Ad<Float,N,Tag>> { enum { value = 1 }; };
  
  template<int ddl, size_t N, class Tag> std::array<adct::Ad<typename Tag::second_type,ddl,typename Tag::first_type>,N> to_adct_residual(ttt::Int<N> const&, Tag)
  {
    std::array<adct::Ad<typename Tag::second_type,ddl,typename Tag::first_type>,N> a;
    a.fill(adct::Ad<typename Tag::second_type,ddl,typename Tag::first_type>(0,0));
    return a;
  }
  
  template<class Jacob, class Ad, int DDL> struct Ad2Jacob
  {
    Jacob& j;
    const Ad& ad;
    
    Ad2Jacob(Jacob& j_, const Ad& ad_):j(j_),ad(ad_){}
    
    template<int I, int J> void operator()()
    {
      std::get<I,J>(j) = std::get<I>(ad).infinite()[J+DDL];
    }
  };
  
  template<int DDL, class J, class Ad> Ad2Jacob<J,Ad,DDL> ad2jacob(J& j, const Ad& ad)
  {
    return Ad2Jacob<J,Ad,DDL>(j,ad);
  }

  template<class Tag> struct AutomaticDerivator
  {
    template<class Fun, size_t K, size_t I, size_t F, class Tuple, class Res, class ... R> static void 
    _unroll2(const Fun& fun, ttt::Int<K> const&, ttt::Int<I> const&, ttt::Int<F> const&, const Tuple& tuple, Res& res, R& ... r)
    {
      auto tmp = to_adct<FunctionInfo<Fun>::ddl,K>(ttt::to_ref(bf::at_c<I>(tuple)),Tag());
      _unroll2(fun,ttt::Int<K+Size<decltype(tmp)>::value>(),ttt::Int<I+1>(),ttt::Int<F>(),tuple,res,r...,ttt::to_ref(bf::at_c<I>(tuple)),tmp._M_elems);
    }
    
    template<class Fun, size_t K, size_t F, class Tuple, class Res, class ... R> static void 
    _unroll2(const Fun& fun, ttt::Int<K> const&, ttt::Int<F> const&, ttt::Int<F> const&, const Tuple&, Res& res, R& ... r)
    {
      fun.functor.automatic(r...,res._M_elems);
    }
    
    template<size_t DDL, class Fun, class Result, class AD, class Tuple, size_t F> static void 
    _unroll(ttt::Int<DDL>,const Fun&, ttt::Int<F>const& , ttt::Int<F> const&, AD&, Result&, const Tuple&) {}

    template<size_t DDL, class Fun, class Result, class AD, class Tuple, size_t I, size_t F> static void 
    _unroll(ttt::Int<DDL>,const Fun& fun, ttt::Int<I> const&, ttt::Int<F> const&, AD& ad, Result& result, const Tuple& tuple)
    {
      auto& jacob = bf::at_c<I>(result).second;
      typedef typename std::decay<decltype(jacob)>::type Jacob;
      static_assert( int(FunctionInfo<Fun>::erreur_size) == int(Rows<Jacob>::value), "AutomaticDerivative : matrix size doesn't match with error size." );
      double_unroll(ttt::Int<Rows<Jacob>::value>(),ttt::Int<Cols<Jacob>::value>(),ad2jacob<DDL>(jacob,ad));
      _unroll(Int<DDL+Size<decltype(bf::at_c<I>(tuple))>::value>(),fun,ttt::Int<I+1>(),ttt::Int<F>(),ad,result,tuple);
    }

    template<class Fonctor, class Tuple, class Jacob>
    static void derive(const Fonctor& fonctor, const Tuple& tuple, Jacob& result)
    {
      static const size_t erreur_size = FunctionInfo<Fonctor>::erreur_size;
      static const size_t ddl = FunctionInfo<Fonctor>::ddl;
      auto residual = to_adct_residual<ddl>(ttt::Int<erreur_size>(),Tag());
      _unroll2(fonctor,ttt::Int<0>(),ttt::Int<0>(),ttt::Int<mpl::size<Tuple>::value>(),tuple,residual);
      _unroll(Int<0>(),fonctor,ttt::Int<0>(),ttt::Int<mpl::size<Jacob>::value>(),residual,result,tuple);
    }
  };
}

#endif
