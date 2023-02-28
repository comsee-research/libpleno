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

#ifndef __OPTIMISATION2_SOLVER_SOLVER2_HPP__
#define __OPTIMISATION2_SOLVER_SOLVER2_HPP__

#include <libv/core/miscmath.hpp>

#include "../ba/bas.hpp"
#include "../bundle/view.hpp"
#include "default_callback.hpp"
#include "policies.hpp"


namespace lma
{
  template<class ... Functors> struct Solver
  {
    typedef View<mpl::vector<Functors...>> Container;
    
    Container bundle;
    
    double lambda;
    double _goodStepLowerScale; ///< lower bound for lambda decrease if a good LM step
    double _goodStepUpperScale; ///< upper bound for lambda decrease if a good LM step
    double v;
    
    double min_rms;
    double eps;
    double rms1,rms2;
    double initial_cost,final_cost;
    std::size_t nb_iteration_interne;
    std::size_t it_interne;
    bool is_better;

    Solver(double lambda_ = -1.0, std::size_t interne = 20, double eps_ = 0.9999):lambda(lambda_),min_rms(1e-30),eps(eps_),rms1(0),rms2(0),nb_iteration_interne(interne) 
    {
      _goodStepUpperScale = 2./3.;
      _goodStepLowerScale = 1./3.;
      v=2.;
    }

    static std::string name() { return std::string("Solver<") + ttt::name<mpl::vector<Functors...>>() + ">"; }
    
    template<class F, class ... Params> Solver& add(const F& f, const Params& ... params)
    {
      bundle.add(f,params...);
      return *this;
    }
    
    template<class AlgoTag = LdltTag<>, class F = default_callbacks_for_solver>
    Solver& solve(const AlgoTag& config = LdltTag<>(), F callbacks = default_callbacks_for_solver())
    {
      if (bundle.nb_obs()==0)
      {
        return *this;
      }

      rms1 = 0;
      rms2 = 0;
      it_interne = 0;
      is_better = true;

      typedef typename SelectAlgo<Container,Container::NbClass,AlgoTag>::type Algorithm;
      Algorithm algo(config);
      
      algo.init(bundle);

      algo.compute_erreur(bundle);

      rms1 = rms2 = initial_cost = algo.get_erreur();

      callbacks.at_begin_bundle_adjustment(*this,algo);
      
      for( ; it_interne < nb_iteration_interne ; ++it_interne)
      {
        if (!update(algo,callbacks)) break;
      }

      final_cost = algo.get_erreur();

      callbacks.at_end_bundle_adjustment(*this,algo);
      return *this;
    }

    template<class Algo, class F>
    bool update(Algo& algo, const F &callbacks)
    {
      callbacks.at_begin_bundle_adjustment_iteration(*this,algo);

      auto save_container = bundle.clone_opt_container();
      rms1 = algo.get_erreur();
      algo.compute(bundle,lambda,is_better);
      double nb_obs = 0;
      std::tie(rms2,nb_obs) = algo.compute_erreur(bundle);

      double rho = 2.0*(rms1-rms2) / (algo.compute_scale(lambda) + 1e-3);

      if(rms2 > rms1)
      {
        algo.restore_erreur();
        bundle.restore_container(save_container);
        is_better = false;
        callbacks.at_end_bundle_adjustment_iteration(*this,algo);
        lambda*=v;
        v *= 2;
      }
      else
      {
        double alpha = 1. - v::pow<3>(2.*rho-1.);
        double scaleFactor = (std::max)(_goodStepLowerScale, (std::min)(alpha, _goodStepUpperScale));
        lambda *= scaleFactor;
        v = 2;

        is_better = true;
        callbacks.at_end_bundle_adjustment_iteration(*this,algo);
        if (stop(nb_obs))
        {
          rms1 = rms2;
          return false;
        }
        rms1 = rms2;
      }
      return true;
    }

    double stop(double nb_obs) const { return (sqrt(rms2/nb_obs) > (eps * sqrt(rms1/nb_obs))) || (rms2 == rms1) || (rms2 < min_rms); }

    double rms() const { return std::min(rms1,rms2); }
  };
}

namespace ttt
{
  template<class A, size_t B, class C> struct Name<lma::SelectAlgo<A,B,C>>
  {
    static const std::string name(){ return lma::SelectAlgo<A,B,C>::name(); }
  };
}
#endif
