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

#ifndef __TTT_MPL_CAT_HPP__
#define __TTT_MPL_CAT_HPP__

#include <boost/mpl/insert_range.hpp>
#include <boost/mpl/end.hpp>

namespace ttt
{
  template<class LP, class LQ> struct Cat : boost::mpl::insert_range<LP,typename boost::mpl::end<LP>::type,LQ> {};
}

#endif

