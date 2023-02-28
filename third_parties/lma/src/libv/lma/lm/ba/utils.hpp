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

#ifndef __LIBV_LMA_OPT2_BA_UTILS_HPP__
#define __LIBV_LMA_OPT2_BA_UTILS_HPP__

#include <libv/lma/global.hpp>
#include <libv/lma/ttt/fusion/at.hpp>
#include <boost/fusion/include/fold.hpp>
#include <boost/fusion/include/make_vector.hpp>
#include <boost/fusion/include/push_back.hpp>
#include <boost/fusion/include/back.hpp>
#include <boost/mpl/int.hpp>
#include <boost/mpl/push_back.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/fusion/include/as_vector.hpp>

namespace lma
{
  struct Count
  {
    typedef std::size_t result_type;

    template<class Key,class Cont, template<class,class> class Pair> result_type operator()(const result_type& prev, const Pair<Key,Cont>& obj) const
    {
      return prev + obj.second.full_size();
    }
  };

  template<class T> std::size_t nb_element(const T& obj)
  {
    return bf::fold(obj,0,Count());
  }
  
  template<class Tuple, int J, class Result>
  Result& size_tuple_(const Tuple&, mpl::int_<J>, mpl::int_<J>, Result& result)
  {
    return result;
  }
  
  template<class Tuple, int I, int J, class Result>
  Result& size_tuple_(const Tuple& tuple, mpl::int_<I>, mpl::int_<J>, Result& result)
  {
    bf::at_c<I+1>(result) = bf::at_c<I>(result) + bf::at_c<I>(tuple).second.full_size();
    return size_tuple_(tuple,mpl::int_<I+1>(),mpl::int_<J>(),result);
  }
  
  template<class Vector, int N> struct TupleSize : TupleSize< typename mpl::push_back<Vector,size_t>::type,N-1> {};
  template<class Vector> struct TupleSize<Vector,0> : br::as_vector<Vector> {};

  template<int End, class Tuple>
  typename TupleSize<mpl::vector<>,End+1>::type
  size_tuple(const Tuple& tuple)
  {
    static_assert( End > 0, " size_tuple : End > 0 ");
    typename TupleSize<mpl::vector<>,End+1>::type result;
    bf::at_c<0>(result) = 0;
    return size_tuple_(tuple,mpl::int_<0>(),mpl::int_<End>(),result);
  }
}

#endif
 
