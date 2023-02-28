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

#ifndef __UTILS_SRC_TTT_TRAITS_NAMING_HPP__
#define __UTILS_SRC_TTT_TRAITS_NAMING_HPP__

#include <string>
#include "../../string/string_utiliy.hpp"
#include <vector>
#include "clement.hpp"
#include <libv/lma/color/console.hpp>
#include <utility>

namespace ttt {

  template<class T, class Enable = void> struct Name
  {
    static std::string name() { return typeid(T).name();}
  };

  template<class T> std::string name()
  {
    return Name<T>::name();
  }

  template<class T> std::string name(T)
  {
    return Name<T>::name();
  }
  
  template<> struct Name<double> { static std::string name() { return "double"; } };
  template<> struct Name<char> { static std::string name() { return "char"; } };
  template<> struct Name<short> { static std::string name() { return "short"; } };
  template<> struct Name<float> { static std::string name() { return "float"; } };
  template<> struct Name<int> { static std::string name() { return "int"; } };
  template<> struct Name<size_t> { static std::string name() { return "size_t"; } };
  template<> struct Name<bool> { static std::string name() { return "bool"; } };
  
  template<class T, size_t N> struct Name<T[N]> { static std::string name() { return ttt::name<T>() + "[" + lma::to_string(N) + "]"; } };

  template<class A, class B> struct Name< std::pair<A,B> >
  {
    static std::string name() { return std::string("std::pair<") + ttt::name<A>() + "," + ttt::name<B>() + ">";}
  };
  
  template<class T, class A> struct Name< std::vector<T,A> >
  {
    static std::string name() { return "std::vector<" + ttt::name<T>() + ">";}
  };

  template<class T> struct Name< T& >
  {
    static std::string name() { return ttt::name<T>() + "&";}
  };

  template<class T> struct Name< T* >
  {
    static std::string name() { return ttt::name<T>() + "*";}
  };

  template<class T> struct Name< const T* >
  {
    static std::string name() { return ttt::name<T>() + " const*";}
  };

  template<class T> struct Name< T const& >
  {
    static std::string name() { return ttt::name<T>() + " const&";}
  };

  template<> struct Name<void>
  {
    static std::string name() { return "void";}
  };

}//! eon ttt



#endif
