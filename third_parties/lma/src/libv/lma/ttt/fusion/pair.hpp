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

#ifndef __LMA_TTT_FUSION_PAIR_HPP__
#define __LMA_TTT_FUSION_PAIR_HPP__

#include <libv/lma/ttt/traits/naming.hpp>
#include <boost/fusion/include/pair.hpp>

namespace ttt
{
  template<class A, class B> struct Name<boost::fusion::pair<A,B>>
  {
    static std::string name() { return color.red() + "bf::pair<" + color.reset() + ttt::name<A>() + "," + ttt::name<B>() + color.red() + ">" + color.reset();}
  };
}

#endif