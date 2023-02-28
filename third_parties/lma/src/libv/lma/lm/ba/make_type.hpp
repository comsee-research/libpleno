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

#ifndef __OPTIMISATION2_BA_MAKE_TYPE_HPP__
#define __OPTIMISATION2_BA_MAKE_TYPE_HPP__

#include <libv/lma/global.hpp>
#include <boost/mpl/vector.hpp>
#include "container.hpp"
#include <libv/lma/ttt/mpl/for.hpp>

namespace lma
{
  typedef bf::pair<mpl::_1,mpl::_1> pair_;

  template<class A, class B, class Float> struct MakeTupleTable
  { typedef bf::pair<bf::pair<A,B>,Table<A,B,Float>> type;};
  
  template<class A, class Float> struct MakeTupleTable<A,A,Float>
  { typedef bf::pair<bf::pair<A,A>,Table<A,A,Float,Symetric>> type;};

  template<class A, class Float> struct VectorToPairStruct  { typedef bf::pair<A,lma::VectorColumn<A,Float>> type; };
  template<class T> struct VectorToKey { typedef T type; };
}

#endif
