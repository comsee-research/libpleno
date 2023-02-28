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

#ifndef __STRING_DISPLAY_HPP__
#define __STRING_DISPLAY_HPP__

#include <vector>
#include <list>
#include <array>
#include <utility>
#include <iostream>

template<class A, class B> std::ostream& operator<<(std::ostream& o, const std::pair<A,B>& pair)
{
  return o << "std::pair(" << pair.first << ";" << pair.second << ")";
}

template<class T, class A> std::ostream& operator<<(std::ostream& o, const std::vector<T,A>& v)
{
//   std::copy(v.begin(),v.end(),std::ostream_iterator<T>(o," "));
  o << "std::vector[";
  for(std::size_t i = 0 ; i < v.size() ; ++i)
    o << std::endl << (i==0?"":",") << v[i] << "|\n";
  o << "]" /*<< std::endl*/;
  return o;
}

template<class T, class A> std::ostream& operator<<(std::ostream& o, const std::list<T,A>& v)
{
  o << "std::list[";
  for(auto it = v.cbegin() ; it != v.cend() ; it++)
    o << (it==v.cbegin()?"":",") << (*it) << "|\n";
  o << "]" /*<< std::endl*/;
  return o;
}
template<class T, std::size_t N> std::ostream& operator<<(std::ostream& o, const std::array<T,N>& array)
{
  o << "std::array[";
  for(std::size_t i = 0 ; i < N ; ++i)
    o << (i==0?"":",") << array[i];
  o << "]" /*<< std::endl*/;
  return o;
}

#endif