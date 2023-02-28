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

#ifndef __LMA_TTT_MPL_FOR_HPP__
#define __LMA_TTT_MPL_FOR_HPP__

#include <libv/lma/global.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/apply.hpp>
#include <boost/mpl/int.hpp>

namespace ttt
{
  template<size_t B, size_t E, class List, class Functor, class Result = mpl::vector<>> struct For : 
      For<
	  B+1,
	  E,
	  List,
	  Functor,
	  typename mpl::apply<
			      Functor,
			      List,
			      mpl::int_<B>,
			      Result
			     >::type
	 > {};
  
  template< size_t E, class L, class F, class R> struct For<E,E,L,F,R> : R {};  

}

#endif