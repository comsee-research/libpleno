
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

#ifndef __TTT_MPL_NAMING_HPP__
#define __TTT_MPL_NAMING_HPP__

#include <libv/lma/global.hpp>
#include "../traits/naming.hpp"
#include <boost/mpl/is_sequence.hpp>
#include <boost/mpl/next.hpp>
#include <boost/mpl/begin.hpp>
#include <boost/mpl/deref.hpp>
#include <boost/mpl/end.hpp>
#include <boost/mpl/int.hpp>
#include <boost/utility/enable_if.hpp>

namespace ttt
{
  template<class A, class B> struct DispTypeVector
  {
    static std::string name()
    {
      static std::string delimitor = boost::is_same<typename boost::mpl::next<A>::type,B>::value ? "" : ",";
      return ttt::name<typename boost::mpl::deref<A>::type>() + delimitor + DispTypeVector<typename boost::mpl::next<A>::type,B>::name();
    }
  };

  template<class A> struct DispTypeVector<A,A>
  {
    static std::string name()
    {
      return ttt::name<typename boost::mpl::deref<A>::type>();
    }
  };

  template<class Vector> struct Name<Vector,typename boost::enable_if<boost::mpl::is_sequence<Vector>>::type>
  {
    static std::string name() { return color.green() + "mpl::vector<" + color.reset() + DispTypeVector<typename boost::mpl::begin<Vector>::type, typename boost::mpl::end<Vector>::type>::name() + color.green() + ">" + color.reset(); }
  };

  template<> struct Name<boost::mpl::void_>
  {
    static std::string name() { return ""; }
  };
  template<int I> struct Name<mpl::int_<I>>
  {
    static std::string name() { return std::string("Int<") + lma::to_string(I) + ">";}
  };

  template<> struct Name<mpl_::na> { static std::string name(){return "mpl::na";}};
}

#endif

