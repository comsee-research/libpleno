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

#ifndef __OPTIMISATION2_FUNCTION_FUNCTION_HPP__
#define __OPTIMISATION2_FUNCTION_FUNCTION_HPP__

#include <libv/lma/ttt/traits/functor_dispatch.hpp>
#include <libv/lma/ttt/mpl/unique_cat.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/find_if.hpp>
#include <boost/mpl/transform.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/remove.hpp>
#include <boost/mpl/deref.hpp>
#include <boost/mpl/assert.hpp>
#include <boost/mpl/int.hpp>
#include <boost/fusion/include/push_back.hpp>
#include <boost/fusion/include/pair.hpp>

#include <libv/lma/lm/trait/size.hpp>
#include <libv/lma/lm/container/container.hpp>


namespace lma
{
  template<class T, class R = void> struct InterpretResidu;
  template<class T> struct InterpretResidu<T&> : mpl::false_ { typedef T type; };
  template<class T, size_t N> struct InterpretResidu<T(&)[N]> : mpl::true_ { typedef std::array<T,N> type; };
  
  template<class T> inline T& to_array(T& array, mpl::false_ const&)
  {
    return array;
  }
  
  template<class T, size_t N> inline typename std::array<T,N>::_AT_Type::_Type& to_array(std::array<T,N>& residual, mpl::true_ const&)
  {
    return residual._M_elems;
  }

  
  
  template<class F> struct Function
  {
    typedef ttt::FunctorTrait<decltype(&F::operator())> FunctorTrait;
    typedef typename FunctorTrait::ParametersType ParametersType;
    typedef ParametersType type;
    typedef InterpretResidu<typename FunctorTrait::Residu> IR;
    typedef typename IR::type ErreurType;

    const F& functor;

    Function(const F& f):functor(f){}

    template<class Input> bool operator()(const Input& input, ErreurType& error) const
    {
      return ttt::call(functor,to_array(error,IR()),input);
    }
  };

  template<class F> struct FunctionInfo
  {
    typedef typename F::ParametersType ParametersType;
    typedef typename F::ErreurType ErreurType;

    template<class A,class B> struct Sum : mpl::plus<A,mpl::int_<Size<B>::value>> {};
    typedef mpl::fold<
          ParametersType,
          mpl::int_<0>,
          Sum<mpl::_1,mpl::_2>
               > DDL;
        
    enum { ddl = DDL::type::value };
    enum { erreur_size = Size<ErreurType>::value };
  };

  template<class F> Function<F> make_function(const F& f) { return Function<F>(f); }
  
  template<class T> struct DoNothing {typedef T type;};
  
  template<class F> struct CreateListArg : Function<F>::ParametersType {};
  
  template<class V, class F> struct CatFunctionParameters : ttt::UniqueCat<V,CreateListArg<F>> {};
  
  template<class L, class F> struct CreateListParam 
  {
    typedef typename     
    mpl::fold<
              L,
              mpl::vector<>,
              CatFunctionParameters<mpl::_1,mpl::_2>
             >::type l;
    typedef typename mpl::apply<F,l>::type type;
  };
  
  template<class L, class F> struct FindOrder
  {
    typedef typename
    mpl::find_if<
                  L,
                  mpl::is_sequence<mpl::_1>
                >::type iterator;
    
    template<class List, class Order> struct Result { typedef typename List::type list_function; typedef typename Order::type list_parameter;};
    
    typedef typename
    mpl::if_<
              boost::is_same<
                              iterator,
                              typename mpl::end<L>::type
                            >,
              Result<DoNothing<L>,CreateListParam<L,F>>,
              Result<mpl::remove<L,typename mpl::deref<iterator>::type>, mpl::apply<F,typename mpl::deref<iterator>::type>>
            >::type type;
  };
}

namespace ttt
{

}
#endif
