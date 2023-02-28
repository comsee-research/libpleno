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

#ifndef __LMA_OPT2_SOLVER_DEFAULT_CALLBACK_HPP__
#define __LMA_OPT2_SOLVER_DEFAULT_CALLBACK_HPP__

#include <libv/lma/global.hpp>

namespace lma
{
  struct default_callbacks_for_solver
  {
    template<class S, class A>
    void at_begin_bundle_adjustment(const S&, const A&) const {}
    
    template<class S, class A>
    void at_begin_bundle_adjustment_iteration(const S&, const A&) const {}
    
    template<class S, class A>
    void at_end_bundle_adjustment_iteration(const S&, const A&) const {}
    
    template<class S, class A>
    void at_end_bundle_adjustment(const S&, const A&) const {}
  };
}

#endif
