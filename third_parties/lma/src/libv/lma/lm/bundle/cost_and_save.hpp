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

#ifndef __OPTIMISATION2_BUNDLE_COST_AND_SAVE_HPP__
#define __OPTIMISATION2_BUNDLE_COST_AND_SAVE_HPP__

#include <cmath>
#include "../function/function.hpp"
#include <libv/lma/ttt/traits/wrap.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/has_key.hpp>
#include "../ba/nan_error.hpp"
#include "../ba/mat.hpp"
#include "../omp/omp.hpp"

#include <libv/lma/ttt/traits/unroll1.hpp>
#include <libv/lma/ttt/traits/rm_all.hpp>
#include <libv/lma/lm/trait/use_estimator.hpp>

namespace std
{
  template<size_t I> const double& get(const double& d)
  {
    static_assert(I==0,"get<I>(double) : I!=0");
    return d;
  }
}

namespace lma
{

  template<class Obs, class Bundle, class VErreur, class Mad> std::pair<double,int> cost_and_save_(const Bundle& bundle, VErreur& errors, const Mad& mad)
  {
    const auto nb_obs = bundle.template at_obs<Obs>().size();
    if (nb_obs==0) return {0,0};
    errors.resize(nb_obs());
    double error = 0;
    size_t cpt = 0;
// #pragma omp parallel for reduction(+:error) if(use_omp())
    for(auto iobs = bundle.template at_obs<Obs>().first() ; iobs < nb_obs ; ++iobs)
    {
      if ((errors[iobs()].second = make_function(bundle.obs(iobs))(bundle.map(iobs),errors[iobs()].first)))
      {
        auto tmp = errors[iobs()].first;
        detail::apply_mestimator_erreur<Obs>(bundle.obs(iobs),tmp,mad);
        error += squared_norm(tmp);
        cpt ++;
      }
    }

    if (is_invalid(error))
    {
      std::cout << " Erreur : " << error << std::endl;
      std::string msg = std::string() + " NAN : cost_and_save in functor " + ttt::name<Obs>() + ".";
      throw NAN_ERROR(msg);
    }

    return std::make_pair(error / 2.0, cpt);
  }


  template<class Bundle, class MapErreur, class Meds> struct CosterSave
  {
    const Bundle& bundle;
    MapErreur& map_erreur;
    const Meds& meds;
    std::pair<double,int> sumcpt;
    
    CosterSave(const Bundle& bundle_, MapErreur& map_erreur_, const Meds& meds_):bundle(bundle_),map_erreur(map_erreur_),meds(meds_),sumcpt({0,0}){}

    template<class Obs> void operator()(ttt::wrap<Obs>)
    {
      auto pair = cost_and_save_<Obs,Bundle>(bundle,map_erreur.template at_key<Obs>(),meds);
      sumcpt.first += pair.first;
      sumcpt.second += pair.second;
    }
  };

  template<class Bundle, class MapErreur, class Meds> std::pair<double,int> cost_and_save(const Bundle& bundle, MapErreur& map_erreur, const Meds& meds)
  {
    CosterSave<Bundle,MapErreur,Meds> coster(bundle,map_erreur,meds);
    mpl::for_each<typename Bundle::ListFunction, ttt::wrap<mpl::placeholders::_1>>(boost::ref(coster));
    return coster.sumcpt;
  }

  
  //////////////
  // MEDIANE  //
  //////////////

  template<class Error> struct _ADDNORM
  {
    Error error;
    std::vector<double>& norms;
    _ADDNORM(Error error_, std::vector<double>& norms_):error(error_),norms(norms_){}
    
    template<size_t I> void operator()(Int<I> const &)
    {
      norms.push_back(std::abs(std::get<I>(error)));
    }
  };
  
  template<class Obs, class Bundle> void cost_and_save_mad_(const Bundle& bundle, std::vector<double>& norms)
  {
    const auto nb_obs = bundle.template at_obs<Obs>().size();
    if (nb_obs!=0) norms.reserve(nb_obs());
// #pragma omp parallel for if(use_omp())
    for(auto iobs = bundle.template at_obs<Obs>().first() ; iobs < nb_obs ; ++iobs)
    {
      typename Function<Obs>::ErreurType error;
      if (make_function(bundle.obs(iobs))(bundle.map(iobs),error))
      {
#ifndef NDEBUG
        if (std::isnan(squared_norm(error)))
        {
          std::cout << " Nan found in " << ttt::name<Obs>() << std::endl;
          throw NAN_ERROR(" NAN : cost_and_save_mad_");
        }
#endif
        ttt::unroll<0,Size<typename ttt::rm_all<decltype(error)>::type>::value>(_ADDNORM<decltype(error)>(error,norms));
      }
    }
  }

  template<class Bundle, class Meds> struct CosterSaveMad
  {
    const Bundle& bundle;
    Meds& meds;//une mediane par foncteur robuste
    
    CosterSaveMad(const Bundle& bundle_, Meds& meds_):bundle(bundle_),meds(meds_){}

    template<class Obs, class Value> void operator()(ttt::wrap<bf::pair<Obs,Value>>)
    {
      if (bundle.template at_obs<Obs>().size()==0) return;
      std::vector<double> norms;
      cost_and_save_mad_<Obs,Bundle>(bundle,norms);
      
      //std::cout << color.bold() << "\n Mad " << ttt::name<Obs>() << " : ";
      bf::at_key<Obs>(meds) = bundle.template at_obs<Obs>()(0).compute(norms);
      //std::cout << color.reset();
    }
  };

  //! pour chaque foncteur
  template<class Meds, class Bundle>
  void cost_and_save_mad(const Bundle& bundle, Meds& meds)
  {
    CosterSaveMad<Bundle,Meds> coster(bundle,meds);
    mpl::for_each<Meds, ttt::wrap<mpl::placeholders::_1>>(boost::ref(coster));
  }
  
}//eon

#endif

