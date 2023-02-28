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

#ifndef __OPTIMISATION2_CONTAINER_RESIDUAL_HPP__
#define __OPTIMISATION2_CONTAINER_RESIDUAL_HPP__

#include <cstddef>
#include "../trait/size.hpp"
#include <array>
#include <libv/lma/ttt/traits/int.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include <cmath>
/*
namespace lma
{
  template<class T, std::size_t N> struct Residual : std::array<T,N>
  {
    static_assert( N!=0, " Residual : N==0 ");
    
    T squared_norm(ttt::Int<N-1>) const
    {
      return std::get<N-1>(*this)*std::get<N-1>(*this);
    }
    
    template<size_t I> T squared_norm(ttt::Int<I>) const
    {
      return std::get<I>(*this)*std::get<I>(*this) + squared_norm(ttt::Int<I+1>());
    }
    
    T squared_norm() const
    {
      return squared_norm(ttt::Int<0>());
    }
    
    T norm() const
    {
      return std::sqrt(squared_norm());
    }
    
    void mul(ttt::Int<N>, T){}
    
    template<size_t I> void mul(ttt::Int<I>, T scalar)
    {
      static_assert( I < N, " Residual::mul I < N ");
      std::get<I>(*this) *= scalar;
      mul(ttt::Int<I+1>(),scalar);
    }
    
    void operator*=(T scalar)
    {
      mul(ttt::Int<0>(),scalar);
    }
    

    
    template<class X, class ... Value> void append(const X& x, const Value& ... value)
    {
      std::get<0>(*this) = x;
      append(ttt::Int<1>(),value...);
    }
    
    private:
      template<class X, class ... Value, size_t I> void append(ttt::Int<I>, const X& x, const Value& ... value)
      {
        std::get<I>(*this) = x;
        append(ttt::Int<I+1>(),value...);
      }
      void append(ttt::Int<N>){}
  };
  
  template<class T, std::size_t N> struct Size<Residual<T,N>> { static const size_t value = N; };

}

*/

namespace ttt
{
  template<class T, size_t N> struct Name<std::array<T,N>> 
  { static std::string name(){ return "array<" + ttt::name<T>() + "," + lma::to_string(N) + ">"; } };
}


#endif
