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

#ifndef __TTT_TRAITS_WRAP_HPP__
#define __TTT_TRAITS_WRAP_HPP__

#include <boost/mpl/placeholders.hpp>

namespace ttt
{
  template<class T> struct wrap {};
  typedef wrap<boost::mpl::_1> wrap_;
}

#endif
