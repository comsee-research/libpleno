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

#ifndef __TTT_FUSION_AT_HPP__
#define __TTT_FUSION_AT_HPP__

#include <boost/fusion/include/at_key.hpp>
#include <libv/lma/global.hpp>

namespace ttt
{
  template<class Map,class Key1, class Key2> struct BinaryAt
  {
    typedef typename br::value_at_key<Map,Key2>::type Map2;
    typedef typename br::value_at_key<Map2,Key1>::type type;
    typedef typename boost::add_reference<type>::type type_ref;
    typedef typename boost::add_const<type>::type const_type;
    typedef typename boost::add_reference<const_type>::type const_ref_type;
  };

  template<class A, class B, class Container> 
  typename BinaryAt<Container,B,A>::const_ref_type at(const Container& container)
  {
    BOOST_MPL_ASSERT((boost::is_same<decltype(bf::at_key<B>(bf::at_key<A>(container))),typename BinaryAt<Container,B,A>::const_ref_type>));
    return bf::at_key<B>(bf::at_key<A>(container));
  }
}

#endif


