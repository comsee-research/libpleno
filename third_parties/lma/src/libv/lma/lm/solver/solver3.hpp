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

#ifndef __OPTIMISATION2_SOLVER_SOLVER3_HPP__
#define __OPTIMISATION2_SOLVER_SOLVER3_HPP__

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
    double min_rms;
    double eps;
    double lambda;
    double min_lambda;
    double rms1,rms2;
    double initial_cost,final_cost;
    std::size_t nb_iteration_interne;
    std::size_t it_interne;
    bool is_better;

    Solver(double lamb = 1e-3, std::size_t interne = 20, double eps_ = 0.9999, double min_lambda_=1e-10):min_rms(1e-30),eps(eps_),lambda(lamb),min_lambda(min_lambda_),rms1(0),rms2(0),nb_iteration_interne(interne) {}

    
    template<class F, class ... Params> Solver& add(const F& f, const Params& ... params)
    {
      bundle.add(f,params...);
      return *this;
    }
    
    template<class AlgoTag = LdltTag<>, class F = default_callbacks_for_solver>
    Solver& solve(const AlgoTag& config = LdltTag<>(), const F &callbacks = default_callbacks_for_solver())
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
      callbacks.at_begin_bundle_adjustment_iteration();

      auto save_container = bundle.clone_opt_container();
      rms1 = algo.get_erreur();
//       double C = 0;
      algo.compute(bundle,lambda,is_better);
      rms2 = algo.compute_erreur(bundle);

//       double& A = rms1;
//       double& B = rms2;
// 
//       double rho = (B-A) / (C-A);
//       
//       std::cout << "\n A : " << rms1 << std::endl;
//       std::cout << " B : " << rms2 << std::endl;
//       std::cout << " C : " << C << std::endl;
//       std::cout << " ared : " << (B-A) << std::endl;
//       std::cout << " pred : " << (C-A) << std::endl;
//       std::cout << color.red() << " rho = " << rho << std::endl << color.reset() << std::endl;
      double nb_obs = bundle.nb_obs();
      if(rms2 > rms1)
      {
        algo.restore_erreur();
        bundle.restore_container(save_container);
        is_better = false;
        callbacks.at_end_bundle_adjustment_iteration(*this);
        lambda *= 10.0;
        if (lambda > 1e30) return false;
      }
      else
      {
        is_better = true;
        callbacks.at_end_bundle_adjustment_iteration(*this);
        if (stop(nb_obs))
        {
          rms1 = rms2;
          return false;
        }
        else
          lambda = std::max(min_lambda,lambda / 10.0);
        rms1 = rms2;
      }
      return true;
    }

    double stop(double nb_obs) const
    {
      return (sqrt(rms2/nb_obs) > (eps * sqrt(rms1/nb_obs))) || (rms2 == rms1) || (rms2 < min_rms);
    }

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
