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

#ifndef __TTT_FUSION_COPY_F_HPP__
#define __TTT_FUSION_COPY_F_HPP__

#include <libv/lma/global.hpp>
#include <boost/fusion/include/tuple.hpp>
#include <boost/fusion/include/begin.hpp>

#include <cassert>

//! Largely inspired by boost::fusion::for_each
//! Copy each element of tuple2 to tuple1 according to the given fonctor
//! Both tuple must have the same size
//! for example see unit-ttt-fusion-copy_f

namespace ttt{
  namespace detail{
  using namespace boost::fusion;
  template<class Iterator1, class Iterator2, class End2, class Fonctor>
  inline void binary_for_each_iterate(Iterator1 , Iterator2 , End2 , const Fonctor& , boost::mpl::true_)  {}

  template<class Iterator1, class Iterator2, class End2, class Fonctor> inline void binary_for_each_iterate
  (Iterator1 it1, Iterator2 it2, End2 end2, const Fonctor& fonctor, boost::mpl::false_)
  {
    fonctor(deref(it1),deref(it2));
    binary_for_each_iterate(next(it1),next(it2),end2,fonctor,
      result_of::equal_to<typename result_of::next<Iterator2>::type, End2>()
      );
  }

  template<class Tuple1, class Tuple2, class Fonctor> inline void binary_for_each(Tuple1& tuple1, const Tuple2& tuple2, const Fonctor& fonctor)
  {
    binary_for_each_iterate(begin(tuple1),begin(tuple2),end(tuple2),fonctor,mpl::false_());
  }

}// eon detail

  template<class Tuple1, class Tuple2, class Fonctor> inline void copy_f(Tuple1& tuple1, const Tuple2& tuple2, const Fonctor& fonctor)
  {
    detail::binary_for_each(tuple1,tuple2,fonctor);
  }

} //eon ttt

#endif


