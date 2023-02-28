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

#ifndef __TTT_TRAITS_FUNCTIONTRAIT_HPP__
#define __TTT_TRAITS_FUNCTIONTRAIT_HPP__

#include <libv/lma/global.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/back.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/pop_back.hpp>
#include <boost/mpl/size.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/type_traits/is_reference.hpp>
#include <type_traits>

namespace ttt
{
  // récupérer les paramètres d'une fonction membre const
  template<class> struct Extract;
  template<class Return, class F, class ...T> struct Extract< Return (F::*) (T...) const >
  {
    typedef Return ReturnType;
    typedef mpl::vector<T...> tmp1;
    typedef typename mpl::transform<tmp1,std::decay<mpl::_1>>::type tmp2;
    typedef typename mpl::back<tmp1>::type Residu;
    typedef typename mpl::pop_back<tmp2>::type Args;
  };

  template<class F> struct FunctorTrait
  {
    typedef F FunctorType;
    typedef Extract<FunctorType> Ext;
    
    typedef typename Ext::ReturnType ReturnType;
    typedef typename Ext::Residu Residu;
    typedef typename Ext::Args ParametersType;

    static const std::size_t size = mpl::size<ParametersType>::value;
    
    static_assert( boost::is_same<ReturnType,bool>::value, " Functor::operator() return type must be bool");
    static_assert( boost::is_reference<Residu>::value, " Functor::operator(param1,...,paramN, Residual& residual) : residual must be given by reference");
    static_assert( size !=0 , "Don't forget parameter or residual : Functor::operator(param1,...,paramN, Residual& residual)");
  };
}// eon

#endif
