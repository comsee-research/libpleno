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

#ifndef __OPTIMISATION2_BA_FILL_HESSIEN_HPP__
#define __OPTIMISATION2_BA_FILL_HESSIEN_HPP__

#include <libv/lma/lm/function/derivative/default_derivative.hpp>
#include <libv/lma/lm/ba/fonctorba.hpp>
#include <libv/lma/lm/omp/omp.hpp>
#include <libv/lma/time/tictoc.hpp>
#include <libv/lma/lm/trait/use_estimator.hpp>

namespace lma
{
  namespace detail
  {
    //! on calcule chaque jacobienne et on met à jour la hessienne.
    //! les erreurs sont pré-calculées
    template<class Tag, class Bundle, class Ba, class MapErreur, class Medianes> struct FillHessien32
    {
      const Bundle& bundle;
      Ba& ba;
      MapErreur& map_erreur;
      const Medianes& medianes;
      
      FillHessien32(const Bundle& bundle_, Ba& ba_, MapErreur& map_erreur_, const Medianes& medianes_)
      : bundle(bundle_),ba(ba_),map_erreur(map_erreur_),medianes(medianes_){}


      template<class Obs/*, class DerivativeTag*/> void sequential(ttt::wrap<Obs> const &)// DerivativeTag = NumericForward || NumericCentral || Analytical
      {
        auto& v_erreur = map_erreur.template at_key<Obs>();
        for(auto iobs = bundle.template at_obs<Obs>().first() ; iobs < bundle.template at_obs<Obs>().size() ; ++iobs)
        {
          if (v_erreur[iobs()].second)
          {
            auto fun = make_function(bundle.obs(iobs));
            auto map = bundle.map(iobs);
            
            typename detail::JacobReturnType<Tag,decltype(fun),decltype(map)>::type jacob;
            derivator<Tag>(fun,map,jacob,v_erreur[iobs()].first);
            const auto& map_indice = bundle.spi2.indices(iobs);// vector<Indice,...>
            detail::apply_mestimator(bundle.obs(iobs),jacob,v_erreur[iobs()].first,medianes);
            auto tie = bf::vector_tie(map_indice,jacob,v_erreur[iobs()].first,bundle,iobs,ba.h,ba.jte);
            static const size_t size = mpl::size<typename ttt::rm_all<decltype(map_indice)>::type>::value;
            ttt::unroll<0,size>(JtJ<size,decltype(tie),typename Ba::MatrixTag>(tie));
          }
        }
      }

      template<class Obs> void operator()(ttt::wrap<Obs> const & wrap)
      {
        sequential(wrap);
      }
    };

    /*
    template<class Derivator, class Bundle, class Ba, class MapErreur> struct FillHessien33
    {
      const Bundle& bundle;
      Ba& ba;
      MapErreur& map_erreur;
      FillHessien33(const Bundle& bundle_, Ba& ba_, MapErreur& map_erreur_):bundle(bundle_),ba(ba_),map_erreur(map_erreur_){}

      template<class Fun, class Map, class Jacob, class Erreur>
      void call_derivative(const Fun& fun, const Map& map, Jacob& jacob, const Erreur& erreur, NumericForward)
      {
        Derivator::derive(fun,map,jacob,erreur);
      }
      
      template<class Fun, class Map, class Jacob, class Erreur, class DerivativeTag>
      void call_derivative(const Fun& fun, const Map& map, Jacob& jacob, const Erreur&, DerivativeTag)// DerivativeTag = NumericCentral || Analytical
      {
        Derivator::derive(fun,map,jacob,DerivativeTag());
      }
      
      template<class Obs, class DerivativeTag> void sequential(DerivativeTag)// DerivativeTag = NumericForward || NumericCentral || Analytical
      {
        auto& v_erreur = map_erreur.template at_key<Obs>();
        auto size = bundle.template at_obs<Obs>().size();
        auto& jacobs = bf::at_key<Obs>(ba.jacob);
        jacobs.resize(size());

        std::cout << " fill jacob " << std::endl;
        for(auto iobs = bundle.template at_obs<Obs>().first() ; iobs < size ; ++iobs)
        {
          if (v_erreur[iobs()].second)
          {
            auto fun = make_function(bundle.obs(iobs));
            auto map = bundle.map(iobs);
            auto& jacob = jacobs[iobs()];
            call_derivative(fun,map,jacob,v_erreur[iobs()].first,DerivativeTag());// map<pair<Key,Jacob>,...>
            const auto& map_indice = bundle.spi2.indices(iobs);// vector<Indice,...>
            auto tie = bf::vector_tie(map_indice,jacob,v_erreur[iobs()].first,bundle,iobs,ba.h,ba.jte);
            static const size_t size = mpl::size<typename ttt::rm_all<decltype(map_indice)>::type>::value;
            ttt::unroll<0,size>(JtJ<size,decltype(tie),typename Ba::MatrixTag>(tie));
          }
        }
      }

      template<class Obs> void operator()(ttt::wrap<Obs>)
      {
        typedef typename 
        boost::mpl::if_<
                      boost::is_convertible<Obs*,Analytical*>,
                      Analytical,
                      typename boost::mpl::if_< 
                                                boost::is_convertible<Obs*,NumericForward*>,
                                                NumericForward,
                                                NumericCentral// default numeric derivative is central
                                              >::type
                     >::type DispatchTag;
        
        sequential<Obs>(DispatchTag());
      }
    };
    
    template<class Derivator, class Bundle, class Ba, class MapErreur>
    void store_jacob_and_fill_hessien(const Bundle& bundle, Ba& ba, MapErreur& map_erreur)
    {
      FillHessien33<Derivator,Bundle,Ba,MapErreur> fh(bundle,ba,map_erreur);
      mpl::for_each<typename Bundle::ListFunction, ttt::wrap<mpl::placeholders::_1>>(fh);
    }*/
    
    template<class Tag, class Bundle, class Ba, class MapErreur, class Medianes>
    void fill_hessien(const Bundle& bundle, Ba& ba, MapErreur& map_erreur, const Medianes& meds)
    {
      FillHessien32<Tag,Bundle,Ba,MapErreur,Medianes> fh(bundle,ba,map_erreur,meds);
      mpl::for_each<typename Bundle::ListFunction, ttt::wrap<mpl::placeholders::_1>>(fh);
    }
  }// eon

}// eon

#endif
