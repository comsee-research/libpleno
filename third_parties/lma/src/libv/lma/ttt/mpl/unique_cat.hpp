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

#ifndef __TTT_MPL_UNIQUE_CAT_HPP__
#define __TTT_MPL_UNIQUE_CAT_HPP__

#include <boost/mpl/deref.hpp>
#include <boost/mpl/push_back.hpp>
#include <boost/mpl/begin.hpp>
#include <boost/mpl/end.hpp>
#include <boost/mpl/contains.hpp>

namespace ttt
{
  namespace detail
  {
    template<class V, class E, class T> struct TryAdd : V { };

    template<class V, class E> struct TryAdd<V,E,boost::mpl::false_> : boost::mpl::push_back<V,E>::type { };

    template<class vector, class begin, class end> struct AddIfNotPresent
    {
      typedef typename boost::mpl::deref<begin>::type current;
      typedef typename TryAdd<vector,current,typename boost::mpl::contains<vector,current>::type>::type type0;
      typedef typename AddIfNotPresent<type0,typename boost::mpl::next<begin>::type, end>::type type;
    };

    template<class vector, class end> struct AddIfNotPresent<vector,end,end> : vector { };
  }

  template<class P, class Q> struct UniqueCat : detail::AddIfNotPresent<P,typename boost::mpl::begin<Q>::type,typename boost::mpl::end<Q>::type>::type { };
}

#endif
