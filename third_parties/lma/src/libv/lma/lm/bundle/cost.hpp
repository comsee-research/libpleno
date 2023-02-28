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

#ifndef __OPTIMISATION2_BUNDLE_COST_HPP__
#define __OPTIMISATION2_BUNDLE_COST_HPP__

#include <cmath>
#include "../function/function.hpp"
#include <libv/lma/ttt/traits/wrap.hpp>
#include <boost/mpl/for_each.hpp>
#include "../ba/nan_error.hpp"
#include "../ba/mat.hpp"
#include "../omp/omp.hpp"

namespace lma
{
  template<class Obs, class Bundle> double cost(const Bundle& bundle)
  {
    const auto nb_obs = bundle.template at_obs<Obs>().size();
    if (nb_obs==0) return 0;

//     std::cout << " cost without save " << std::endl;
    
    double total = 0;

//     #pragma omp parallel for reduction(+:total) if(use_omp())
    for(auto iobs = bundle.template at_obs<Obs>().first() ; iobs < nb_obs ; ++iobs)
    {
      //total += (make_function(bundle.obs(iobs))(bundle.map(iobs))).squaredNorm() / 2;
      auto pair_residu = make_function(bundle.obs(iobs))(bundle.map(iobs));
      if (pair_residu.second)
        total += ( pair_residu ).first.squaredNorm();
    }

    if (std::isnan(total))
      throw NAN_ERROR(" NAN : cost_and_save");
    return total/2.0;
  }

  template<class Bundle> struct Coster
  {
    const Bundle& bundle;
    double sum;
    Coster(const Bundle& bundle_):bundle(bundle_),sum(0){}

    template<class T> void operator()(ttt::wrap<T>)
    {
      sum += cost<T,Bundle>(bundle);
    }
  };

  // total square error
  template<class Bundle> double cost(const Bundle& bundle)
  {
    Coster<Bundle> coster(bundle);
    mpl::for_each<typename Bundle::ListeToObs, ttt::wrap<mpl::placeholders::_1>>(boost::ref(coster));
    return coster.sum;
  }

  template<class ListeObs, class Bundle> double costs(const Bundle& bundle)
  {
    Coster<Bundle> coster(bundle);
    mpl::for_each<ListeObs, ttt::wrap<mpl::placeholders::_1>>(boost::ref(coster));
    return coster.sum;
  }
  
  // root mean square
  template<class Bundle> double rms(const Bundle& bundle)
  {
    if (bundle.nb_obs()==0) return 0;
    return  sqrt( cost(bundle) / (double)(bundle.nb_obs()));
  }

  // mean square error
  template<class Bundle> double mse(const Bundle& bundle)
  {
    if (bundle.nb_obs()==0) return 0;
    return   cost(bundle) * 2.0 / (double)(bundle.nb_obs());
  }

}//eon

#endif

