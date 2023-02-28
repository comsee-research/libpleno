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

#ifndef __OPTIMISATION2_FUNCTION_DERIVATIVE_DETAIL_DERIVATIVE_HPP__
#define __OPTIMISATION2_FUNCTION_DERIVATIVE_DETAIL_DERIVATIVE_HPP__

#include <cmath>
#include <limits>

#include "../../trait/size.hpp"
#include "../function.hpp"
#include "../../container/container.hpp"
#include <boost/mpl/push_back.hpp>
#include <boost/fusion/mpl/push_back.hpp>
#include <boost/fusion/include/at_c.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/fusion/include/deref.hpp>
#include <boost/fusion/include/pair.hpp>
#include <boost/fusion/include/map.hpp>
#include <libv/lma/ttt/traits/to_ref.hpp>
#include <libv/lma/ttt/fusion/at.hpp>

namespace lma
{
  namespace detail
  {
    //! ToKey allow Sequence to be an associative container
    template<class Key> struct ToKey { typedef Key type; };
    template<template<class,class> class Pair, class Key, class Value> struct ToKey < Pair<Key,Value> > { typedef Key type; };

    template<class _1, class N, class Tag> struct Jac 
    {
     typedef  boost::fusion::pair<
                            typename ToKey<_1>::type,
                            typename lma::ContainerOption<
                                                          Tag,
                                                          lma::Size<N>::value,
                                                          lma::Size<typename ToKey<_1>::type>::value
                                                         >::Matrix> type;
    };
    template<class Tag, class F, class Sequence> struct JacobReturnType :
      boost::mpl::transform<
			      Sequence,
			      Jac<
				  mpl::_1,
				  v::numeric_tag<lma::Size<typename F::ErreurType>::value>,
				  Tag
				  >
			    > {};
  }//! eon

}//! eon

#endif
