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

#ifndef __OPTIMISATION2_TRAIT_SIZE_HPP__
#define __OPTIMISATION2_TRAIT_SIZE_HPP__

#include <cstddef>
#include <utility>
#include <libv/core/tag.hpp>
#include <type_traits>
#include <boost/mpl/bool.hpp>
#include <array>

namespace lma
{
  template<class T, class = void> struct Size { enum {value = -1}; };

  template<class T, size_t N> struct Size< T[N] > { enum { value = N }; };
  
  template<class T> struct Size<T&> : Size<T> {};
  template<class T> struct Size<const T> : Size<T> {};
  template<class T> struct Size<T*> : Size<T> {};
  template<class T> struct Size<const T*> : Size<T> {};
  template<class T> struct Size<const T&> : Size<T> {};

  template<class T, size_t N> struct Size< std::array<T,N> > { enum { value = N } ; };

  template<class T> struct Size< std::pair<T,bool> > : Size<T> {};

  template<> struct Size< float >          { enum { value = 1 }; };
  template<> struct Size< int >            { enum { value = 1 }; };
  template<> struct Size< double >         { enum { value = 1 }; };
  template<int I> struct Size<v::numeric_tag<I>>  { enum { value = I }; };
  
  template<class X, class R = void> struct SizeIsDefined : boost::mpl::false_ {};
  template<class X> struct SizeIsDefined< X, typename std::enable_if<Size<X>::value!=-1>::type > : boost::mpl::true_ {};
}

#endif
