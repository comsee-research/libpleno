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

#ifndef __LMA_OPTIMISATION2_TRAIT_FAKE_HPP__
#define __LMA_OPTIMISATION2_TRAIT_FAKE_HPP__

#include "size.hpp"
#include "accessor.hpp"

namespace lma
{
  template<class T, class Tag = void> struct Fake
  {
    T _;
    T* ptr;
    Fake(T& t):_(t),ptr(&t){}
    ~Fake() { *ptr = _; }
    void update() { *ptr = _; }
  };
  
  template<class T, class Tag> struct Size<Fake<T,Tag>>{ static const size_t value = Size<T>::value; };
  
  template<class T, class Tag, class Float, int N> void apply_increment(Fake<T,Tag>& fake, const Eigen::Matrix<Float,N,1>& delta)
  {
    apply_increment(fake._,delta);
  }
  
  template<int I,class T, class Tag, class Float> void apply_small_increment(Fake<T,Tag>& fake, Float d, v::numeric_tag<I> nt)
  {
    apply_small_increment(fake._,d,nt);
  }
}

namespace ttt
{
  template<class T> struct Name<lma::Fake<T>> { static std::string name() { return "Fake<" + ttt::name<T>() + ">"; } };
}

#endif
