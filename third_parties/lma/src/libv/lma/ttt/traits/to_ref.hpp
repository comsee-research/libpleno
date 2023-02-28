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

#ifndef __TTT_TRAITS_TOREF_HPP__
#define __TTT_TRAITS_TOREF_HPP__

namespace ttt
{
  template<class T> struct ToRef
  {
    typedef T& ref;
    typedef const T& const_ref;
  };

  template<class T> struct ToRef<T*>: ToRef<T> {};
  

  template<class T> inline T& to_ref(T& obj) { return obj;}
  template<class T> inline T& to_ref(T* obj) { return *obj;}

}

#endif
