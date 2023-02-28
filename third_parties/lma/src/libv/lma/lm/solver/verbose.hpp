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

#ifndef __OPTIMISATION2_SOLVER_VERBOSE_HPP__
#define __OPTIMISATION2_SOLVER_VERBOSE_HPP__

/**

\file
\author Alexis Wilhelm (2013)
\copyright 2013 Institut Pascal

*/

#include <libv/lma/time/tictoc.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include "default_callback.hpp"
#include <boost/format.hpp>
#include <boost/fusion/include/for_each.hpp>


namespace lma {

template<class> struct LevMar;

namespace internal {

template<class Policy>
static void print_stats(const LevMar<Policy> &a, const boost::format &format)
{
  std::cerr
  << boost::format(format) % "Pre-Process" % a.preprocess
  << boost::format(format) % "Residual evaluations" % a.residual_evaluations
  << boost::format(format) % "Jacobian evaluations" % a.jacobian_evaluations
  << boost::format(format) % "Solver normal eq." % a.norm_eq_
  ;
}

}

struct enable_verbose_output: default_callbacks_for_solver
{
  template<class Solver, class Algorithm>
  void at_begin_bundle_adjustment(Solver& s, const Algorithm& ) const
  {
    clock_total.tic();
    clock_iteration.tic();

    std::cerr
    << std::endl
    << "Bundle adjustment using:"
    << "\e[33m"
    << std::endl
    << ttt::name<Algorithm>()
    << "\e[36m"
    << std::endl;

    boost::fusion::for_each(s.bundle.opt_container.map(), print_var());
    boost::fusion::for_each(s.bundle.fonction_container.map(), print_observation());

    std::cerr
    << "\e[m"
    << std::endl
    << boost::format("%3s  %7s  %10s  %11s  %10s  %11s  %8s  %8s")
      % "#"
      % "D"
      % "Cost"
      % "dCost"
      % "RMS"
      % "dRMS"
      % "IT"
      % "TT"
    << std::endl
    << std::string(82, '-')
    << std::endl;

    print_iteration(s,s.bundle,"");
  }

  template<class Solver, class Algorithm>
  void at_begin_bundle_adjustment_iteration(const Solver&, const Algorithm&) const
  {
    clock_iteration.tic();
  }

  template<class Solver, class Algorithm>
  void at_end_bundle_adjustment_iteration(const Solver& s, const Algorithm&) const
  {
    this->print_iteration(s,s.bundle,s.is_better ? "\e[32m" : "\e[31m");
  }

  template<class Solver, class Algo>
  void at_end_bundle_adjustment(const Solver& s, const Algo& algo) const
  {
    static const boost::format format("\e[36m%-21s: %g\e[m\n");

    std::cerr
    << std::endl
    << boost::format(format) % "Initial" % s.initial_cost
    << boost::format(format) % "Final" % s.final_cost
    << boost::format(format) % "Change" % (s.initial_cost - s.final_cost)
    << std::endl
    ;

    internal::print_stats(algo, format);

    std::cerr
    << boost::format(format) % "Total time" % clock_total.toc()
    ;
  }

  double total_time() const { return clock_total.toc(); }
  
private:

  mutable utils::Tic<true> clock_total, clock_iteration;

  struct print_var { template<template<class, class> class Pair, class Key, class Value> void operator()(const Pair<Key, Value> &o) const {
//     std::cerr << boost::format("%s (%d)\n") % ttt::name<Key>() % o.second.size();
    std::cerr << boost::format("%s (%d)\n") % ttt::name<Key>() %  o.second.size();
  }};

  struct print_observation { template<template<class, class> class Pair, class Key, class Value> void operator()(const Pair<Key, Value> &o) const {
    std::cerr << boost::format("%s (\e[1m%d\e[21m)\n") % ttt::name<Key>() % o.second.size();
  }};

  template<class Solver, class Bundle>
  void print_iteration(const Solver& s, const Bundle& b, const char *color) const
  {
    std::cerr
    << boost::format("%s%3d  %7.1g  %10.5g  %+11.5g  %10.5g  %+11.5g  %8.3g  %8.3g\e[m")
      % color
      % s.it_interne
      % s.lambda
      % s.rms2
      % (s.rms2 - s.rms1)
      % sqrt(s.rms2 / b.nb_obs())
      % (sqrt(s.rms2 / b.nb_obs()) - sqrt(s.rms1 / b.nb_obs()))
      % clock_iteration.toc()
      % clock_total.toc()
    << std::endl;
  }
};

  struct minimal_verbose : default_callbacks_for_solver
  {
    mutable utils::Tic<true> clock_total;
    
    template<class S, class A>
    void at_begin_bundle_adjustment(const S&, const A&) const
    {
      clock_total.tic();
      std::cerr
        << std::endl
        << "\e[33m"
        << ttt::name<A>()
        << "\e[36m : ";
    }
    
    template<class Solver, class Algo>
    void at_end_bundle_adjustment(const Solver& s, const Algo&) const
    {
      std::cerr 
        << " From " << s.initial_cost 
        << " to " << s.final_cost 
        << " in " << clock_total.toc() 
        << " sec.(" << s.it_interne 
        << " it)." << std::endl;
      // static const boost::format format("\e[36m%-21s: %g\e[m, ");
      // std::cerr << boost::format(format) % "Initial" % s.initial_cost;;
      // std::cerr << boost::format(format) % "Final cost" % s.final_cost;
//       std::cerr << boost::format(format) % "Total time" % clock_total.toc()
      ;
    }
  };
}

#endif
